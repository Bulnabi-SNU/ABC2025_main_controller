__author__ = "Chaewon Yun"
__contact__ = "gbll0305@gmail.com"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleGlobalPosition
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

# import message for YOLOv5
# from my_msgs import ~

# import other libraries
import os
import time
import serial
import logging
import numpy as np
import pymap3d as p3d
from datetime import datetime, timedelta

#import cv2

class VehicleController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        """
        0. Configure QoS profile for publishing and subscribing
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        """
        1. Constants
        """
        # given constants
        self.camera_to_center = 0.5                             # distance from camera to center of the vehicle
        self.landing_height = 5.0                               # prepare auto landing at 5m
        self.corridor_radius = 2.0

        # time period
        self.time_period = 0.05                                  # 20 Hz

        # acceptance constants
        self.mc_acceptance_radius = 0.3
        self.nearby_acceptance_radius = 30
        self.offboard_acceptance_radius = 10.0                   # mission -> offboard acceptance radius
        self.transition_acceptance_angle = 0.8                   # 0.8 rad = 45.98 deg
        self.landing_acceptance_angle = 0.8                      # 0.8 rad = 45.98 deg
        self.heading_acceptance_angle = 0.1                      # 0.1 rad = 5.73 deg

        # bezier curve constants
        self.very_fast_vmax = 7.0
        self.fast_vmax = 5.0
        self.slow_vmax = 2.5
        self.very_slow_vmax = 1.0
        self.max_acceleration = 9.81 * np.tan(10 * np.pi / 180)  # 10 degree tilt angle
        self.mc_start_speed = 0.0001
        self.mc_end_speed = 0.0001
        self.bezier_threshold_speed = 0.7
        self.bezier_minimum_time = 3.0

        # alignment constants
        self.yaw_speed = 0.1                                    # 0.1 rad = 5.73 deg



        """
        2. state and substate
        """
        # state description
        # ready2flight -> takeoff -> seek -> approach -> connect -> rtl -> land
        self.state = 'ready2flight'

        # substate description
        self.substate = 'before flight'


        """
        3. State variables
        """
        # vehicle status
        self.auto = 0                           # 0: manual, 1: auto
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        # vehicle position, velocity, and yaw
        self.pos = np.array([0.0, 0.0, 0.0])        # local
        self.pos_gps = np.array([0.0, 0.0, 0.0])    # global
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        
        # goal position and yaw
        self.goal_position = None
        self.goal_yaw = None

        # Bezier curve
        self.num_bezier = 0
        self.bezier_counter = 0
        self.bezier_points = None


        """
        4. Create Subscribers
        """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile
        )

        """
        5. Create Publishers
        """
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )

        """
        6. timer setup
        """
        self.offboard_heartbeat = self.create_timer(self.time_period, self.offboard_heartbeat_callback)
        self.vehicle_state_publisher_timer = self.create_timer(self.time_period, self.vehicle_state_publisher_callback)
        self.main_timer = self.create_timer(self.time_period, self.main_timer_callback)
    

    """
    Services
    """   
    # def print(self, *args, **kwargs):
    #     print(*args, **kwargs)
    #     self.logger.info(*args, **kwargs)
    
    def convert_global_to_local_waypoint(self, home_position_gps):
        self.home_position = self.pos   # set home position
        self.start_yaw = self.yaw     # set initial yaw
        # for i in range(1, 8):
        #     # gps_WP = [lat, lon, rel_alt]
        #     wp_position = p3d.geodetic2ned(self.gps_WP[i][0], self.gps_WP[i][1], self.gps_WP[i][2] + home_position_gps[2],
        #                                     home_position_gps[0], home_position_gps[1], home_position_gps[2])
        #     wp_position = np.array(wp_position)
        #     self.WP.append(wp_position)
        # self.WP.append(np.array([0.0, 0.0, -self.landing_height]))  # landing position
        # self.print(f'WP: {self.WP}\n')

    def generate_bezier_curve(self, xi, xf, vmax):
        # reset counter
        self.bezier_counter = 0

        # total time calculation
        total_time = np.linalg.norm(xf - xi) / vmax * 2      # Assume that average velocity = vmax / 2.     real velocity is lower then vmax
        if total_time <= self.bezier_minimum_time:
            total_time = self.bezier_minimum_time

        direction = np.array((xf - xi) / np.linalg.norm(xf - xi))
        vf = self.mc_end_speed * direction
        if np.linalg.norm(self.vel) < self.bezier_threshold_speed:
            vi = self.mc_start_speed * direction
        else:
            vi = self.vel
        self.bezier_counter = int(1 / self.time_period) - 1

        point1 = xi
        point2 = xi + vi * total_time / 3
        point3 = xf - vf * total_time / 3
        point4 = xf

        # Bezier curve
        self.num_bezier = int(total_time / self.time_period)
        bezier = np.linspace(0, 1, self.num_bezier).reshape(-1, 1)
        bezier = point4 * bezier**3 +                             \
                3 * point3 * bezier**2 * (1 - bezier) +           \
                3 * point2 * bezier**1 * (1 - bezier)**2 +        \
                1 * point1 * (1 - bezier)**3
        
        return bezier
    
    def run_bezier_curve(self, bezier_points, goal_yaw=None):
        if goal_yaw is None:
            goal_yaw = self.yaw
        
        if self.bezier_counter < self.num_bezier:
            self.publish_trajectory_setpoint(
                position_sp = bezier_points[self.bezier_counter],
                yaw_sp = self.yaw + np.sign(np.sin(goal_yaw - self.yaw)) * self.yaw_speed
            )
            self.bezier_counter += 1
        else:
            self.publish_trajectory_setpoint(
                position_sp = bezier_points[-1],        # last point (goal position)
                yaw_sp = self.yaw + np.sign(np.sin(goal_yaw - self.yaw)) * self.yaw_speed
            )

    def run_turning_yaw(self, yaw_speed):
        self.publish_trajectory_setpoint(
                yaw_sp = self.yaw + yaw_speed
            )
    
    def get_braking_position(self, pos, vel):
        braking_distance = (np.linalg.norm(vel))**2 / (2 * self.max_acceleration)
        return pos + braking_distance * vel / np.linalg.norm(vel)
    
    def get_bearing_to_next_waypoint(self, now, next):
        now2d = now[0:2]
        next2d = next[0:2]
        direction = (next2d - now2d) / np.linalg.norm(next2d - now2d) # NED frame
        yaw = np.arctan2(direction[1], direction[0])
        return yaw
    
    def find_indices_below_threshold(self, arr, threshold):
        return [i for i, value in enumerate(arr) if value < threshold]
    
    def intersection(self, arr1, arr2):
        return [x for x in arr1 if x in arr2]
    
    def gimbal_reboot(self):
        if is_jetson():
            data_fix = bytes([0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x80, 0x00, 0x01])
            data_crc = crc_xmodem(data_fix)
            packet = bytearray(data_fix + data_crc)
            self.ser.write(packet)

    """
    Callback functions for the timers
    """    
    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(position=True)

    def main_timer_callback(self):
        if self.state == 'ready2flight':
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # self.print('\n\n<< final_taean_left >>\n\n')
                # self.print("Offboard mode requested\n")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.convert_global_to_local_waypoint(self.pos_gps)
                self.state = 'seek'
                self.substate = 'takeoff'
                # self.print('\n[state : 0 -> 1]\n')

        elif self.state == 'seek':
            if self.substate == 'takeoff':
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                        param1=1.0, # main mode
                        param2=6.0  # offboard
                    )
                elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.goal_yaw = self.get_bearing_to_next_waypoint(self.WP[1], self.WP[2])
                    self.goal_position = np.array([0.0, 0.0, 25.0]) # rising altitude to 25m
                    self.bezier_points = self.generate_bezier_curve(self.pos, self.goal_position, self.fast_vmax)
                    self.substate = 'rising'

            elif self.substate == 'rising':
                # rising to 25m && tuning yaw
                if np.linalg.norm(self.pos - self.goal_position) < self.mc_acceptance_radius:
                    self.run_turning_yaw(self.yaw_speed)
                else:
                    self.run_bezier_curve(self.bezier_points, self.goal_yaw)
                    self.run_turning_yaw(self.yaw_speed)

                if True: # when drone find the balloon
                    self.state = 'approach'
                    self.substate = 'generate path'

        elif self.state == 'approach':
            if self.substate == 'generate path':
                # generate path to the balloon
                self.substate = 'approaching'
            
            elif self.substate == 'approaching':
                # follow the path to the balloon
                self.substate = 'generate path'
            
            elif self.substate == 'stop':
                # stop when the vehicle is near the balloon
                self.state = 'connect'
                self.substate = 'calculate distance'

        elif self.state == 'connect':
            if self.substate == 'calculate distance':
                # calculate distance to the balloon
                self.substate = 'align'
            
            elif self.substate == 'align':
                self.substate = 'connect'

            elif self.substate == 'connect':
                self.state = 'rtl'
                self.substate = 'generate path'
    
        elif self.state == 'rtl':
            if self.substate == 'generate path':
                # generate path to the home position
                self.substate = 'returning'
            
            elif self.substate == 'returning':
                # follow the path to the home position
                self.substate = 'generate path'
            
            elif self.substate == 'stop':
                # stop when the vehicle is near the home position
                self.state = 'land'
                self.substate = 'land'

                    
        elif self.state == 'land':                
            if self.substate == 'land':
                self.print("Landing requested\n")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = -2
        
        else:
            if self.state != -2:
                self.print("Error: Invalid state\n")
                self.destroy_node()
                rclpy.shutdown()
            else:
                self.print("Mission completed")
                self.destroy_node()
                rclpy.shutdown()
            

    """
    Callback functions for subscribers.
    """        
    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg
    
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.vel = np.array([msg.vx, msg.vy, msg.vz])
        self.yaw = msg.heading
        if self.state != -1:
            # set position relative to the home position after takeoff
            self.pos = self.pos - self.home_position

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position = msg
        self.pos_gps = np.array([msg.lat, msg.lon, msg.alt])


    """
    Functions for publishing topics.
    """
    def publish_vehicle_command(self, command, **kwargs):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", float('nan'))
        msg.param2 = kwargs.get("param2", float('nan'))
        msg.param3 = kwargs.get("param3", float('nan'))
        msg.param4 = kwargs.get("param4", float('nan'))
        msg.param5 = kwargs.get("param5", float('nan'))
        msg.param6 = kwargs.get("param6", float('nan'))
        msg.param7 = kwargs.get("param7", float('nan'))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def publish_offboard_control_mode(self, **kwargs):
        msg = OffboardControlMode()
        msg.position = kwargs.get("position", False)
        msg.velocity = kwargs.get("velocity", False)
        msg.acceleration = kwargs.get("acceleration", False)
        msg.attitude = kwargs.get("attitude", False)
        msg.body_rate = kwargs.get("body_rate", False)
        msg.thrust_and_torque = kwargs.get("thrust_and_torque", False)
        msg.direct_actuator = kwargs.get("direct_actuator", False)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_trajectory_setpoint(self, **kwargs):
        msg = TrajectorySetpoint()
        # position setpoint is relative to the home position
        msg.position = list( kwargs.get("position_sp", np.nan * np.zeros(3)) + self.home_position )
        msg.velocity = list( kwargs.get("velocity_sp", np.nan * np.zeros(3)) )
        msg.yaw = kwargs.get("yaw_sp", float('nan'))
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)



def is_jetson():
    try:
        with open('/etc/nv_tegra_release', 'r') as f:
            return True
    except FileNotFoundError:
        return False
    


def main(args = None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()
    rclpy.spin(vehicle_controller)

    vehicle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
