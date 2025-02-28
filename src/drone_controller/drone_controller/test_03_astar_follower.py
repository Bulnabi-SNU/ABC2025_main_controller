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

from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped


# import required libraries
import os
import time
import serial
import logging
import numpy as np
import pymap3d as p3d
from datetime import datetime, timedelta

# import required libraries;'
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import yaml
import numpy as np

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
        # time period
        self.time_period = 0.05                                  # 20 Hz

        # acceptance constants
        self.mc_acceptance_radius = 0.3
        self.offboard_acceptance_radius = 10.0                   # mission -> offboard acceptance radius
        self.heading_acceptance_angle = 0.1                      # 0.1 rad = 5.73 deg

        # bezier curve constants
        self.approach_vmax = 3.0
        self.max_acceleration = 9.81 * np.tan(10 * np.pi / 180)  # 10 degree tilt angle
        self.mc_start_speed = 0.0001
        self.mc_end_speed = 0.0001
        self.bezier_threshold_speed = 0.7
        self.bezier_minimum_time = 3.0

        # alignment constants
        self.yaw_speed = 0.1                                    # 0.1 rad = 5.73 deg
        self.yaw_acceptance = 0.15                              # 0.15 rad = 8.59 deg


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

        # check target direction
        self.target_buffer = [None] * 10
        self.target_direction = 0
        self.delta_yaw = 0
        self.delta_pitch = 0
        self.camera_height = 0
        self.camera_width = 0
        self.K_inverse = 0

        # approach periods
        self.approach_time_threshold = 2 * int(1 / self.time_period)  # 2 seconds
        self.approach_time_counter = 0

        # astar path
        self.astar_path = Path()
        self.astar_path_flag = False
        self.astar_start_pos = np.array([0.0, 0.0, 0.0])
        self.astar_path_step = 0
        self.astar_path_length = 0



        """
        4. Load Data
        """
        # load camera calibration data
        yaml_file = 'src/camera_calibration/zed_calibration_formatted.yaml'  # Path to the YAML file
        with open(yaml_file, 'r') as file:
            camera_data = yaml.full_load(file)
        
        self.camera_height = float(camera_data['height'])  # Height(float)
        self.camera_width = float(camera_data['width'])   # Width(float)
        self.K_inverse = np.linalg.inv(np.array(camera_data['k'], dtype=np.float32))  # k(np.array)



        """
        5. Create Subscribers
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

        self.astar_path_subscriber = self.create_subscription(
            Path, '/planned_path', self.astar_path_callback, 10
        )
        self.astar_start_pos_subscriber = self.create_subscription(
            PointStamped, '/start_pos', self.astar_start_pos_callback, 10
        )

        """
        6. Create Publishers
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
        7. timer setup
        """
        self.offboard_heartbeat = self.create_timer(self.time_period, self.offboard_heartbeat_callback)
        self.main_timer = self.create_timer(self.time_period, self.main_timer_callback)
        self.vehicle_state_timer = self.create_timer(self.time_period, self.vehicle_state_callback)
    

    """
    Services
    """ 
    def convert_global_to_local_waypoint(self, home_position_gps):
        self.home_position = self.pos   # set home position
        self.start_yaw = self.yaw     # set initial yaw
    
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
    
    def run_astar_path(self, goal_yaw=None):
        if goal_yaw is None:
            goal_yaw = self.yaw
            
        if self.astar_path_flag:
            if self.astar_path_step < self.astar_path_length:
                self.astar_path_step += 1
                self.publish_trajectory_setpoint(
                    position_sp = np.array([
                        self.astar_path[self.astar_path_step].pose.position.x,
                        self.astar_path[self.astar_path_step].pose.position.y,
                        self.astar_path[self.astar_path_step].pose.position.z
                    ]),
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
    
    def update_target_direction(self):
        # calculate target direction
        target2d = np.array([(self.xmin+self.xmax)/(2*self.screen_width)*self.camera_width, (self.ymin+self.ymax)/(2*self.screen_height)*self.camera_height, 1], dtype=np.float32)
        target3d = np.dot(self.K_inverse, target2d)
        self.delta_yaw = np.arctan(target3d[0] / target3d[2])
        self.delta_pitch = np.arctan(target3d[1] / target3d[2])

        # update target buffer
        self.target_buffer.pop(0)
        self.target_buffer.append(target3d * 20)

        # calculate average target direction without None
        valid_vectors = [vec for vec in self.target_buffer if vec is not None]
        if valid_vectors:
            self.target_direction = np.mean(valid_vectors, axis=0)
        else:
            self.target_direction = np.zeros(3)  # 기본값


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
                    self.bezier_points = self.generate_bezier_curve(self.pos, self.goal_position, self.approach_vmax)
                    self.substate = 'rising'
                    print('rising')

            elif self.substate == 'rising':
                # rising to 25m && turn yaw
                if np.linalg.norm(self.pos - self.goal_position) < self.mc_acceptance_radius:
                    self.publish_trajectory_setpoint(pos_sp=self.goal_position, yaw_sp=self.turning_yaw(self.yaw_speed))
                else:
                    self.run_bezier_curve(self.bezier_points, self.turning_yaw(self.yaw_speed))

                if self.astar_path_flag:
                    print('detected\nready to approach')
                    self.state = 'approach'
                    self.substate = 'approaching'
        
        elif self.state == 'approach':            
            if self.substate == 'approaching':
                self.run_astar_path()
            
            # when the drone approaches the balloon near enough to calculate distance between the drone and the balloon
            if self.depth_activation or self.path_step >= self.astar_path_length:
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
        # update target direction. when the drone finds the balloon
        self.update_target_direction()
    
    def depth_activation_callback(self, msg):
        self.depth_activation = msg
    
    def astar_path_callback(self, msg):
        self.astar_path = msg.poses
        self.astar_path_flag = True
        self.astar_path_length = len(self.astar_path)
        self.astar_path_step = 0
    
    def astar_start_pos_callback(self, msg):
        self.astar_start_pos = np.array(msg)


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
