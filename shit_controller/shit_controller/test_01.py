# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import required msgs

# import px4_msgs
"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleGlobalPosition
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

# import msgs for object detection
"""msgs for subscription"""
from my_msgs.msg import YoloDetection, DepthActivated, VehicleState


# import required libraries
import os
import time
import serial
import logging
import numpy as np
import pymap3d as p3d
from datetime import datetime, timedelta

# import required libraries
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import yaml
import numpy as np

#import cv2

class VehicleController(Node):

    def __init__(self):
        super().__init__('shit_controller')

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

        # time period
        self.time_period = 0.05                                  # 20 Hz

        # acceptance constants
        self.mc_acceptance_radius = 0.3
        self.offboard_acceptance_radius = 10.0                   # mission -> offboard acceptance radius
        self.heading_acceptance_angle = 0.1                      # 0.1 rad = 5.73 deg

        # bezier curve constants
        self.fast_vmax = 5.0
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
        self.home_position = np.array([0.0, 0.0, 0.0])
        
        # goal position and yaw
        self.goal_position = None
        self.goal_yaw = None

        # Bezier curve
        self.num_bezier = 0
        self.bezier_counter = 0
        self.bezier_points = None

        # Initialize variables
        self.depth_activation = False

        # for obstacle detection
        self.yolo_subscription = 0
        self.obstacle_label = ""
        self.screen_width = 0
        self.screen_height = 0
        self.xmin = 0
        self.xmax = 0
        self.ymin = 0
        self.ymax = 0


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
        self.yolo_subscriber = self.create_subscription(
            YoloDetection, '/yolo_detection', self.yolo_callback, qos_profile
        )
        self.depth_activation_subscriber = self.create_subscription(
            DepthActivated, '/depth_activated', self.depth_activation_callback, qos_profile
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
        self.vehicle_state_publisher = self.create_publisher(
            VehicleState, '/vehicle_state', qos_profile
        )

        """
        6. timer setup
        """
        self.offboard_heartbeat = self.create_timer(self.time_period, self.offboard_heartbeat_callback)
        self.main_timer = self.create_timer(self.time_period, self.main_timer_callback)
        self.vehicle_state_timer = self.create_timer(self.time_period, self.vehicle_state_callback)


        """
        6. Logging
        """
    

    """
    Services
    """   
    def print(self, *args, **kwargs):
        print(*args, **kwargs)
        self.logger.info(*args, **kwargs)
    
    def convert_global_to_local_waypoint(self, home_position_gps):
        self.home_position = self.pos   # set home position
        self.start_yaw = self.yaw     # set initial yaw

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

    def turning_yaw(self, yaw_speed):
        return self.yaw + yaw_speed
    
    def get_braking_position(self, pos, vel):
        braking_distance = (np.linalg.norm(vel))**2 / (2 * self.max_acceleration)
        return pos + braking_distance * vel / np.linalg.norm(vel)
    
    def get_bearing_to_next_waypoint(self, now, next):
        now2d = now[0:2]
        next2d = next[0:2]
        direction = (next2d - now2d) / np.linalg.norm(next2d - now2d) # NED frame
        yaw = np.arctan2(direction[1], direction[0])
        return yaw


    """
    Callback functions for the timers
    """    
    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(position=True)
    
    def vehicle_state_callback(self):
        msg = VehicleState()
        msg.state = self.state
        msg.substate = self.substate
        self.vehicle_state_publisher.publish(msg)

    def main_timer_callback(self):
        if self.state == 'ready2flight':
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                print('\n\n<< test_01 >>\n\n')
                print("Offboard mode requested\n")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.convert_global_to_local_waypoint(self.pos_gps)
                self.state = 'seek'
                self.substate = 'takeoff'
                print('takeoff')
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
                    self.goal_position = np.array([0.0, 0.0, -25.0]) # rising altitude to 25m
                    self.bezier_points = self.generate_bezier_curve(self.pos, self.goal_position, self.fast_vmax)
                    self.substate = 'rising'
                    print('rising')

            elif self.substate == 'rising':
                # rising to 25m && turn yaw
                if np.linalg.norm(self.pos - self.goal_position) < self.mc_acceptance_radius:
                    self.publish_trajectory_setpoint(pos_sp=self.goal_position, yaw_sp=self.turning_yaw(self.yaw_speed))
                else:
                    self.run_bezier_curve(self.bezier_points, self.turning_yaw(self.yaw_speed))

                if self.obstacle_label == "ladder": # when drone finds the balloon
                    print('detected\nready to land')
                    self.state = 'land'
                    self.substate = 'land'
                    
        elif self.state == 'land':                
            if self.substate == 'land':
                print("Landing requested\n")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = -2
        
        else:
            if self.state != -2:
                print("Error: Invalid state\n")
                self.destroy_node()
                rclpy.shutdown()
            else:
                print("Mission completed")
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

    def yolo_callback(self, msg):
        self.yolo_subscription = msg
        self.obstacle_label = msg.label
        self.screen_width = msg.screen_width
        self.screen_height = msg.screen_height
        self.xmin = msg.xmin
        self.xmax = msg.xmax
        self.ymin = msg.ymin
        self.ymax = msg.ymax
    
    def depth_activation_callback(self, msg):
        self.depth_activation = msg


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

    def publish_vehicle_state(self, **kwargs):
        msg = VehicleState()
        msg.state = self.state
        msg.substate = self.substate
        self.vehicle_state_publisher.publish(msg)



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
