#!/usr/bin/env python3

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import magicdog_y1_python as magicbot


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


robot: Optional[magicbot.MagicRobot] = None
running = True


def fmt_status(status) -> str:
    return f"{status.code} {status.message}"


def signal_handler(signum, _frame):
    global running, robot
    logging.info("Received signal %s, stopping", signum)
    running = False
    if robot is not None:
        try:
            robot.shutdown()
        except Exception:
            pass
    raise SystemExit(1)


def ensure_ok(status, step: str) -> None:
    if status.code != magicbot.ErrorCode.OK:
        raise RuntimeError(f"{step} failed: {fmt_status(status)}")


def stop_motion(controller) -> None:
    command = magicbot.JoystickCommand()
    command.left_x_axis = 0.0
    command.left_y_axis = 0.0
    command.right_x_axis = 0.0
    command.right_y_axis = 0.0
    controller.send_joystick_command(command)


def hold_command(controller, left_x: float, left_y: float, right_x: float, right_y: float, duration_s: float, hz: float = 20.0) -> None:
    period_s = 1.0 / max(hz, 1.0)
    deadline = time.time() + max(duration_s, 0.0)
    while running and time.time() < deadline:
        command = magicbot.JoystickCommand()
        command.left_x_axis = left_x
        command.left_y_axis = left_y
        command.right_x_axis = right_x
        command.right_y_axis = right_y
        controller.send_joystick_command(command)
        time.sleep(period_s)
    stop_motion(controller)


def do_mapping_motion(motion_controller, axis_scale: float, turn_scale: float) -> None:
    logging.info("Mapping motion: recovery stand")
    ensure_ok(
        motion_controller.set_motion_id(magicbot.MotionId.MOTION_RECOVER_STAND, 10000),
        "set recovery stand",
    )
    time.sleep(1.0)

    logging.info("Mapping motion: switch to terrain walk")
    ensure_ok(
        motion_controller.set_walk_gait_id(magicbot.WalkGaitId.WALK_GAIT_TERRAIN, 10000),
        "set walk gait",
    )
    time.sleep(1.0)

    logging.info("Mapping motion: rotate left")
    hold_command(motion_controller, 0.0, 0.0, -turn_scale, 0.0, 2.5)
    time.sleep(0.5)

    logging.info("Mapping motion: rotate right")
    hold_command(motion_controller, 0.0, 0.0, turn_scale, 0.0, 2.5)
    time.sleep(0.5)

    logging.info("Mapping motion: small forward")
    hold_command(motion_controller, 0.0, axis_scale, 0.0, 0.0, 1.0)
    time.sleep(0.5)

    logging.info("Mapping motion: small backward")
    hold_command(motion_controller, 0.0, -axis_scale, 0.0, 0.0, 1.0)
    time.sleep(0.5)


def resolve_saved_map_path(map_root: Path, map_name: str) -> Path:
    target_root = map_root / map_name
    if not target_root.is_dir():
        raise RuntimeError(f"map root not found: {target_root}")

    candidates = [path for path in target_root.iterdir() if path.is_dir()]
    if not candidates:
        raise RuntimeError(f"no saved map directory under: {target_root}")

    latest = max(candidates, key=lambda path: path.stat().st_mtime)
    if not (latest / "map").exists():
        raise RuntimeError(f"saved map directory missing map subdir: {latest}")
    return latest


def wait_for_nav_result(controller, timeout_s: float, poll_s: float = 0.5):
    deadline = time.time() + timeout_s
    last_status = None
    while running and time.time() < deadline:
        status, nav_status = controller.get_nav_task_status()
        ensure_ok(status, "get nav task status")
        if last_status != nav_status.status:
            logging.info(
                "Nav status: id=%s status=%s error_code=%s error_desc=%s",
                nav_status.id,
                nav_status.status,
                nav_status.error_code,
                nav_status.error_desc,
            )
            last_status = nav_status.status
        if nav_status.status == magicbot.NavStatusType.END_SUCCESS:
            return
        if nav_status.status in (
            magicbot.NavStatusType.END_FAILED,
            magicbot.NavStatusType.CANCEL,
        ):
            raise RuntimeError(
                f"navigation ended unsuccessfully: status={nav_status.status} error_code={nav_status.error_code} error_desc={nav_status.error_desc}"
            )
        time.sleep(poll_s)

    try:
        controller.cancel_nav_task()
    except Exception:
        pass
    raise RuntimeError(f"navigation timeout after {timeout_s:.1f}s")


def make_pose(x: float, y: float, yaw: float):
    pose = magicbot.Pose3DEuler()
    pose.position = [x, y, 0.0]
    pose.orientation = [0.0, 0.0, yaw]
    return pose


def main() -> int:
    global robot

    parser = argparse.ArgumentParser(description="Minimal SLAM + navigation example for A2203.")
    parser.add_argument("--local-ip", default="192.168.54.111")
    parser.add_argument("--map-name", default=f"minimal_map_{int(time.time())}")
    parser.add_argument("--map-root", default="/home/eame/cust_para/maps")
    parser.add_argument("--nav-x", type=float, default=0.3)
    parser.add_argument("--nav-y", type=float, default=0.0)
    parser.add_argument("--nav-yaw", type=float, default=0.0)
    parser.add_argument("--axis-scale", type=float, default=0.15)
    parser.add_argument("--turn-scale", type=float, default=0.20)
    parser.add_argument("--nav-timeout", type=float, default=30.0)
    parser.add_argument("--save-timeout-ms", type=int, default=20000)
    parser.add_argument("--skip-motion", action="store_true")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    robot = magicbot.MagicRobot()
    logging.info("model: %s", magicbot.get_robot_model())
    logging.info("sdk_version: %s", robot.get_sdk_version())
    logging.info("local_ip: %s", args.local_ip)
    logging.info("map_name: %s", args.map_name)

    try:
        if not robot.initialize(args.local_ip):
            raise RuntimeError("robot.initialize failed")

        ensure_ok(robot.connect(), "robot.connect")
        ensure_ok(
            robot.set_motion_control_level(magicbot.ControllerLevel.HighLevel),
            "set high level control",
        )

        motion_controller = robot.get_high_level_motion_controller()
        slam_nav_controller = robot.get_slam_nav_controller()

        if not motion_controller.initialize():
            raise RuntimeError("high level motion controller initialize failed")
        if not slam_nav_controller.initialize():
            raise RuntimeError("slam nav controller initialize failed")

        ensure_ok(
            motion_controller.set_robot_mode(magicbot.RobotMode.ROBOT_MODE_REMOTE, 10000),
            "set robot mode remote",
        )
        stop_motion(motion_controller)

        logging.info("Switch to SLAM mapping mode")
        ensure_ok(
            slam_nav_controller.activate_slam_mode(magicbot.SlamMode.MAPPING, "", 10000),
            "activate mapping mode",
        )
        ensure_ok(slam_nav_controller.start_mapping(10000), "start mapping")
        time.sleep(1.0)

        if not args.skip_motion:
            do_mapping_motion(motion_controller, args.axis_scale, args.turn_scale)
        else:
            logging.info("Skip mapping motion")

        logging.info("Save map")
        ensure_ok(
            slam_nav_controller.save_map(args.map_name, args.save_timeout_ms),
            "save map",
        )
        time.sleep(2.0)

        map_path = resolve_saved_map_path(Path(args.map_root), args.map_name)
        logging.info("Resolved map path: %s", map_path)

        logging.info("Switch to localization mode")
        ensure_ok(
            slam_nav_controller.activate_slam_mode(
                magicbot.SlamMode.LOCALIZATION, str(map_path), 10000
            ),
            "activate localization mode",
        )
        time.sleep(2.0)

        logging.info("Init pose to origin")
        ensure_ok(
            slam_nav_controller.init_pose(make_pose(0.0, 0.0, 0.0), 10000),
            "init pose",
        )
        time.sleep(2.0)

        status, localization_info = slam_nav_controller.get_current_localization_info()
        ensure_ok(status, "get current localization info")
        logging.info(
            "Localization: is_localization=%s pose=(%.3f, %.3f, %.3f)",
            localization_info.is_localization,
            localization_info.pose.position[0],
            localization_info.pose.position[1],
            localization_info.pose.orientation[2],
        )

        logging.info("Switch to navigation mode")
        ensure_ok(
            slam_nav_controller.activate_nav_mode(
                magicbot.NavMode.GRID_MAP, str(map_path), 10000
            ),
            "activate nav mode",
        )
        time.sleep(2.0)

        target = magicbot.NavTarget()
        target.id = 1
        target.frame_id = "map"
        target.goal = make_pose(args.nav_x, args.nav_y, args.nav_yaw)

        logging.info(
            "Send nav target: x=%.3f y=%.3f yaw=%.3f",
            args.nav_x,
            args.nav_y,
            args.nav_yaw,
        )
        ensure_ok(slam_nav_controller.set_nav_target(target, 10000), "set nav target")

        wait_for_nav_result(slam_nav_controller, args.nav_timeout)
        logging.info("Navigation finished successfully")
        return 0

    except Exception as exc:
        logging.error("%s", exc)
        return 1
    finally:
        if robot is not None:
            try:
                stop_motion(robot.get_high_level_motion_controller())
            except Exception:
                pass
            try:
                robot.get_slam_nav_controller().cancel_nav_task()
            except Exception:
                pass
            try:
                robot.disconnect()
            except Exception:
                pass
            try:
                robot.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
