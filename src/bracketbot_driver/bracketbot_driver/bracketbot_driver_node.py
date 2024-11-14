import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from . import setup_odrive
import time

class BracketbotDriverNode(Node):

    def __init__(self):
        super().__init__('bracketbot_driver')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.timer = self.create_timer(1./400., self.timer_callback)
        
        self.motor_controller = setup_odrive.motor_controller
        self.left_torque = 0
        self.right_torque = 0
        self.last_timestamp = time.time()

    def __exit__(self):
        try:
            self.motor_controller.disable_watchdog_left()
            self.motor_controller.disable_watchdog_right()
            self.motor_controller.stop_left()
            self.motor_controller.stop_right()
        except:
            self.get_logger().error('Error stopping motors during cleanup')

    def __del__(self):
        # Stop the motors when the node is destroyed
        try:
            self.motor_controller.disable_watchdog_left()
            self.motor_controller.disable_watchdog_right()
            self.motor_controller.stop_left()
            self.motor_controller.stop_right()
        except:
            self.get_logger().error('Error stopping motors during cleanup')

    def timer_callback(self):
        if time.time() - self.last_timestamp > 0.1:
            self.left_torque = 0
            self.right_torque = 0

        try:
            # Apply torques
            self.motor_controller.set_torque_nm_left(self.left_torque)
            self.motor_controller.set_torque_nm_right(self.right_torque)
            if self.left_torque != 0 or self.right_torque != 0:
                self.get_logger().info(f"left_torque: {self.left_torque:.2f}, right_torque: {self.right_torque:.2f}")
        except:
            self.get_logger().error('Error setting torques')
            setup_odrive.reset_and_initialize_motors()
            return

    def listener_callback(self, msg):
        self.get_logger().info('Received velocity command - linear: x=%f, y=%f, z=%f angular: x=%f, y=%f, z=%f' % 
                              (msg.linear.x, msg.linear.y, msg.linear.z,
                               msg.angular.x, msg.angular.y, msg.angular.z))
        self.last_timestamp = time.time()

        # Limit torques to MAX_TORQUE
        if msg.linear.x > 0: # forward
            self.left_torque = np.clip(msg.linear.x, 0, setup_odrive.MAX_TORQUE)
            self.right_torque = np.clip(msg.linear.x, 0, setup_odrive.MAX_TORQUE)
        elif msg.linear.x < 0: # backward
            self.left_torque = np.clip(msg.linear.x, -setup_odrive.MAX_TORQUE, 0)
            self.right_torque = np.clip(msg.linear.x, -setup_odrive.MAX_TORQUE, 0)
        elif msg.angular.z < 0: # turn left
            self.left_torque = np.clip(-msg.angular.z, 0, setup_odrive.MAX_TORQUE)
            self.right_torque = np.clip(msg.angular.z, -setup_odrive.MAX_TORQUE, 0)
        elif msg.angular.z > 0: # turn right
            self.left_torque = np.clip(-msg.angular.z, -setup_odrive.MAX_TORQUE, 0)
            self.right_torque = np.clip(msg.angular.z, 0, setup_odrive.MAX_TORQUE)
        else:
            self.left_torque = 0
            self.right_torque = 0


def main(args=None):
    rclpy.init(args=args)
    bracketbot_driver_node = BracketbotDriverNode()

    rclpy.spin(bracketbot_driver_node)

    bracketbot_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
