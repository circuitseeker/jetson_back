#!/usr/bin/env python3
import sys
import os

# Add the tt_package directory to the Python path
sys.path.append(os.path.expanduser('~/tt_package/src/tt_robot_control/src/SMS_STS/TTRobot'))

from servo_control import init_servos, move_servos

def main():
    # Initialize servos (using the USB serial device we found)
    if not init_servos('/dev/ttyACM0'):
        print("Failed to initialize servos")
        return

    # Move servos to specified angles (90, 180, 180)
    if move_servos(90, 180, 180):
        print("Successfully moved servos to angles: 90, 180, 180")
    else:
        print("Failed to move servos")

if __name__ == "__main__":
    main() 