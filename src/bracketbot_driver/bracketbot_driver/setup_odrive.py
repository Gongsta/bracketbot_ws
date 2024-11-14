import json
import time
from .odrive_uart import ODriveUART, reset_odrive

reset_odrive()  # Reset ODrive before initializing motors
time.sleep(1)   # Wait for ODrive to reset

MAX_TORQUE = 2.5  # Nm, adjust based on your motor's specifications
MAX_SPEED = 0.4  # m/s, set maximum linear speed

# Initialize motors
# Read motor directions from saved file
import os

try:
    print(f"Current directory: {os.getcwd()}")
    with open('/home/gongster/quickstart/motor_dir.json', 'r') as f:
        motor_dirs = json.load(f)
        left_dir = motor_dirs['left']
        right_dir = motor_dirs['right']
except Exception as e:
    raise Exception("Error reading /share/bracketbot_driver/motor_dir.json")

motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=left_dir, dir_right=right_dir)
motor_controller.start_left()
motor_controller.enable_torque_mode_left()
motor_controller.start_right()
motor_controller.enable_torque_mode_right()
motor_controller.set_speed_rpm_left(0)
motor_controller.set_speed_rpm_right(0)

def reset_and_initialize_motors():
    global motor_controller
    reset_odrive()
    time.sleep(1)  # Give ODrive time to reset
    try:
        motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=left_dir, dir_right=right_dir)
        motor_controller.clear_errors_left()
        motor_controller.clear_errors_right()
        motor_controller.enable_torque_mode_left()
        motor_controller.enable_torque_mode_right()
        motor_controller.start_left()
        motor_controller.start_right()
        print("Motors re-initialized successfully.")
    except Exception as e:
        print(f"Error re-initializing motors: {e}")
