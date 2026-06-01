#!/usr/bin/env python3

import logging
import signal
import sys
import termios
import time
import tty
from typing import Optional

import magicdog_y1_python as magicbot

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

robot: Optional[magicbot.MagicRobot] = None
running = True


def signal_handler(signum, frame):
    global running
    logging.info("Received signal %s, exiting", signum)
    running = False


def print_help():
    logging.info("High-level motion example")
    logging.info("  1 recovery stand        2 terrain walk")
    logging.info("  3 load walk             4 crouch")
    logging.info("  W/A/S/D move            T/G turn")
    logging.info("  X stop                  ? help")
    logging.info("  ESC exit")


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def controller():
    return robot.get_high_level_motion_controller()


def set_motion(motion_id):
    status = controller().set_motion_id(motion_id, 10000)
    if status.code != magicbot.ErrorCode.OK:
        logging.error("set_motion_id failed: code=%s message=%s", status.code, status.message)
        return False
    return True


def set_walk(walk_gait_id):
    status = controller().set_walk_gait_id(walk_gait_id, 10000)
    if status.code != magicbot.ErrorCode.OK:
        logging.error("set_walk_gait_id failed: code=%s message=%s", status.code, status.message)
        return False
    return True


def send_joystick(left_x, left_y, right_x, right_y):
    command = magicbot.JoystickCommand()
    command.left_x_axis = left_x
    command.left_y_axis = left_y
    command.right_x_axis = right_x
    command.right_y_axis = right_y
    controller().send_joystick_command(command)


def main():
    global robot
    signal.signal(signal.SIGINT, signal_handler)

    logging.info("Robot model: %s", magicbot.get_robot_model())
    robot = magicbot.MagicRobot()

    try:
        if not robot.initialize("192.168.54.111"):
            logging.error("Failed to initialize robot SDK")
            return -1

        status = robot.connect()
        if status.code != magicbot.ErrorCode.OK:
            logging.error("Failed to connect: code=%s message=%s", status.code, status.message)
            return -1

        status = robot.set_motion_control_level(magicbot.ControllerLevel.HighLevel)
        if status.code != magicbot.ErrorCode.OK:
            logging.error("Failed to switch to high-level control: %s", status.message)
            return -1

        if not controller().initialize():
            logging.error("Failed to initialize high-level motion controller")
            return -1

        print_help()
        while running:
            key = getch()
            if key == "\x1b":
                break
            if key == "1":
                set_motion(magicbot.MotionId.MOTION_RECOVER_STAND)
            elif key == "2":
                set_walk(magicbot.WalkGaitId.WALK_GAIT_TERRAIN)
            elif key == "3":
                set_walk(magicbot.WalkGaitId.WALK_GAIT_LOAD)
            elif key == "4":
                set_motion(magicbot.MotionId.MOTION_CROUCH)
            elif key.upper() == "W":
                send_joystick(0.0, 1.0, 0.0, 0.0)
            elif key.upper() == "A":
                send_joystick(-1.0, 0.0, 0.0, 0.0)
            elif key.upper() == "S":
                send_joystick(0.0, -1.0, 0.0, 0.0)
            elif key.upper() == "D":
                send_joystick(1.0, 0.0, 0.0, 0.0)
            elif key.upper() == "T":
                send_joystick(0.0, 0.0, -1.0, 0.0)
            elif key.upper() == "G":
                send_joystick(0.0, 0.0, 1.0, 0.0)
            elif key.upper() == "X":
                send_joystick(0.0, 0.0, 0.0, 0.0)
            elif key == "?":
                print_help()
            time.sleep(0.05)

    except Exception as exc:
        logging.error("Example failed: %s", exc)
        return -1
    finally:
        if robot:
            try:
                robot.get_high_level_motion_controller().shutdown()
                robot.disconnect()
                robot.shutdown()
            except Exception as exc:
                logging.error("Cleanup failed: %s", exc)

    return 0


if __name__ == "__main__":
    sys.exit(main())
