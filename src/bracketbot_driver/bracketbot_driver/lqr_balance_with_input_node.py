#!/usr/bin/env python3
import time
import traceback
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from .imu import FilteredLSM6DS3
from .odrive_uart import ODriveUART, reset_odrive
from .lqr import LQR_gains
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


# WHEEL_RADIUS = 6.5 * 0.0254 / 2  # 6.5 inches diameter converted to meters
WHEEL_RADIUS = 0.1546 / 2 # 154.6 mm diameter converted to meters
WHEEL_DIST = 0.235  # 23.5 cm from wheel to center
YAW_RATE_TO_MOTOR_TORQUE = (WHEEL_DIST / WHEEL_RADIUS) * 0.1  # Assuming a rough torque constant
MOTOR_TURNS_TO_LINEAR_POS = WHEEL_RADIUS * 2 * np.pi
RPM_TO_METERS_PER_SECOND = WHEEL_RADIUS * 2 * np.pi / 60
MAX_TORQUE = 2.5  # Nm, adjust based on your motor's specifications
MAX_SPEED = 0.4  # m/s, set maximum linear speed


class BracketBotBalanceNode(Node):
    def __init__(self):
        super().__init__('bracketbot_balance_node')

        # ROS parameters
        self.declare_parameter('control_rate', 400.0)
        self.control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / self.control_rate

        # ROS publishers
        self.state_pub = self.create_publisher(Twist, 'robot_state', 10)

        # ROS subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.zero_angle_sub = self.create_subscription(
            Float64,
            'zero_angle_adjust',
            self.zero_angle_callback,
            10)

        # Initialize components
        self.imu = FilteredLSM6DS3()
        self.imu.calibrate()
        self.setup_controllers()
        self.setup_motors()
        
        # Initialize state variables
        self.zero_angle = 0.0
        self.desired_vel = 0.0
        self.desired_yaw_rate = 0.0
        self.desired_yaw = 0.0
        self.desired_pos = 0.0
        self.last_timestamp = time.time()
        
        # Create timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('BracketBot Balance Node initialized')

    def setup_controllers(self):
        self.K_balance = LQR_gains(
            Q_diag=[100,100,1,10,10,1],
            R_diag=[1, 1]
        )
        self.K_drive = LQR_gains(
            Q_diag=[1,100,0.1,10,1,10],
            R_diag=[0.1, 1]
        )
        self.get_logger().info(f'Balance gains:\n{self.K_balance.round(2)}')
        self.get_logger().info(f'Drive gains:\n{self.K_drive.round(2)}')

    def setup_motors(self):
        reset_odrive()
        time.sleep(1)

        try:
            with open('/home/gongster/quickstart/motor_dir.json', 'r') as f:
                motor_dirs = json.load(f)
                self.left_dir = motor_dirs['left']
                self.right_dir = motor_dirs['right']
        except Exception as e:
            self.get_logger().error(f'Error reading motor_dir.json: {e}')
            raise

        self.motor_controller = ODriveUART(
            port='/dev/ttyAMA1',
            left_axis=1,
            right_axis=0,
            dir_left=self.left_dir,
            dir_right=self.right_dir
        )
        self.initialize_motors()

    def initialize_motors(self):
        self.motor_controller.start_left()
        self.motor_controller.enable_torque_mode_left()
        self.motor_controller.start_right()
        self.motor_controller.enable_torque_mode_right()
        self.motor_controller.set_speed_rpm_left(0)
        self.motor_controller.set_speed_rpm_right(0)

        # Get initial positions
        try:
            l_pos = self.motor_controller.get_position_turns_left()
            r_pos = self.motor_controller.get_position_turns_right()
            self.start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
            self.start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST)
        except Exception as e:
            self.get_logger().error(f'Error reading initial motor positions: {e}')
            self.reset_and_initialize_motors()

    def reset_and_initialize_motors(self):
        reset_odrive()
        time.sleep(1)
        try:
            self.motor_controller = ODriveUART(
                port='/dev/ttyAMA1',
                left_axis=1,
                right_axis=0,
                dir_left=self.left_dir,
                dir_right=self.right_dir
            )
            self.initialize_motors()
            self.get_logger().info('Motors re-initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Error re-initializing motors: {e}')

    def cmd_vel_callback(self, msg):
        self.desired_vel = np.clip(msg.linear.x, -MAX_SPEED, MAX_SPEED)
        self.desired_yaw_rate = msg.angular.z

    def zero_angle_callback(self, msg):
        self.zero_angle = msg.data
        self.get_logger().info(f'Zero angle adjusted to: {self.zero_angle:.2f}')

    def control_loop(self):
        try:
            # If no command for too long, stop new commands
                    # If no command for too long, start ramping down the desired velocities

            time_since_last_cmd = time.time() - self.last_timestamp

            if time_since_last_cmd > 0.1:
                self.get_logger().debug("No recent commands, ramping down desired velocities.")
                # Implement ramp-down towards zero
                self.desired_vel = 0.0
                self.desired_yaw_rate = 0.0

            self.desired_yaw += 3 * self.desired_yaw_rate * self.dt
            self.desired_pos += 3 * self.desired_vel * self.dt
            # Get current state
            current_pitch = self.imu.robot_angle()
            current_yaw_rate = -self.imu.gyro_RAW[2]
            current_pitch_rate = self.imu.gyro_RAW[0]

            # Get motor state
            r_vel = self.motor_controller.get_speed_rpm_right() * RPM_TO_METERS_PER_SECOND
            l_vel = self.motor_controller.get_speed_rpm_left() * RPM_TO_METERS_PER_SECOND
            l_pos = self.motor_controller.get_position_turns_left()
            r_pos = self.motor_controller.get_position_turns_right()
            
            self.get_logger().info(f'Left position: {l_pos:.3f}, Right position: {r_pos:.3f}')

            current_vel = (l_vel + r_vel) / 2
            current_pos = ((l_pos + r_pos) / 2) * MOTOR_TURNS_TO_LINEAR_POS - self.start_pos
            current_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST) - self.start_yaw

            # Publish state
            state_msg = Twist()
            state_msg.linear.x = current_vel
            state_msg.angular.z = current_yaw_rate
            self.state_pub.publish(state_msg)

            # Calculate control
            current_state = np.array([
                current_pos, current_vel, current_pitch*np.pi/180,
                current_pitch_rate, current_yaw, current_yaw_rate
            ])
            
            desired_state = np.array([
                self.desired_pos, self.desired_vel, self.zero_angle*np.pi/180,
                0, self.desired_yaw, self.desired_yaw_rate
            ])
            self.get_logger().info(f'Current State: {current_state}')
            self.get_logger().info(f'Desired State: {desired_state}')


            state_error = (current_state - desired_state).reshape((6,1))
            
            # Choose controller based on desired motion
            if self.desired_vel != 0.0 or self.desired_yaw_rate != 0:
                if self.desired_vel != 0.0:
                    state_error[0,0] = 0
                if self.desired_yaw_rate != 0:
                    state_error[4,0] = 0
                C = -self.K_drive @ state_error
            else:
                C = -self.K_balance @ state_error

            # Calculate motor torques
            D = np.array([[0.5,0.5],[0.5,-0.5]])
            left_torque, right_torque = (D @ C).squeeze()

            # Apply torque limits
            left_torque = np.clip(left_torque, -MAX_TORQUE, MAX_TORQUE)
            right_torque = np.clip(right_torque, -MAX_TORQUE, MAX_TORQUE)

            # Send commands to motors
            self.motor_controller.set_torque_nm_left(left_torque)
            self.motor_controller.set_torque_nm_right(right_torque)

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            self.reset_and_initialize_motors()

def main(args=None):
    rclpy.init(args=args)
    node = BracketBotBalanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Node error: {traceback.format_exc()}')
    finally:
        # Cleanup
        node.motor_controller.stop_left()
        node.motor_controller.stop_right()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
