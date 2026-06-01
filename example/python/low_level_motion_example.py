#!/usr/bin/env python3

import logging
import signal
import sys
import time
from typing import Optional

import magicdog_y1_python as magicbot

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

robot: Optional[magicbot.MagicRobot] = None
running = True
body_imu_counter = 0
leg_state_counter = 0


def signal_handler(signum, frame):
    global running
    logging.info("Received signal %s, exiting", signum)
    running = False


def body_imu_callback(imu_data):
    global body_imu_counter
    if body_imu_counter % 1000 == 0:
        logging.info("Body IMU timestamp=%d", imu_data.timestamp)
        logging.info("Body IMU orientation=%s", imu_data.orientation)
    body_imu_counter += 1


def leg_state_callback(joint_state):
    global leg_state_counter
    if leg_state_counter % 1000 == 0 and joint_state.joints:
        joint = joint_state.joints[0]
        logging.info("Leg state joint0 pos=(%s,%s) vel=%s", joint.posH, joint.posL, joint.vel)
    leg_state_counter += 1


def make_prepare_command(joint_num):
    command = magicbot.JointCommand()
    for _ in range(joint_num):
        joint = magicbot.SingleJointCommand()
        joint.operation_mode = 200
        joint.pos = 0.0
        joint.vel = 0.0
        joint.toq = 0.0
        joint.kp = 0.0
        joint.kd = 0.0
        command.joints.append(joint)
    return command


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

        status = robot.set_motion_control_level(magicbot.ControllerLevel.LowLevel)
        if status.code != magicbot.ErrorCode.OK:
            logging.error("Failed to switch to low-level control: %s", status.message)
            return -1

        controller = robot.get_low_level_motion_controller()
        if not controller.initialize():
            logging.error("Failed to initialize low-level motion controller")
            return -1

        controller.subscribe_body_imu(body_imu_callback)
        controller.subscribe_leg_state(leg_state_callback)

        leg_command = make_prepare_command(magicbot.LEG_JOINT_NUM)

        while running:
            controller.publish_leg_command(leg_command)
            time.sleep(0.002)

    except Exception as exc:
        logging.error("Example failed: %s", exc)
        return -1
    finally:
        if robot:
            try:
                robot.get_low_level_motion_controller().shutdown()
                robot.disconnect()
                robot.shutdown()
            except Exception as exc:
                logging.error("Cleanup failed: %s", exc)

    return 0


if __name__ == "__main__":
    sys.exit(main())
