#!/usr/bin/env python3.11

import argparse
import os
import sys
import time

import magicdog_y1_python as magicbot


def fmt_status(status) -> str:
    return f"{status.code} {status.message}"


def require_ok(status, step: str) -> None:
    if status.code != magicbot.ErrorCode.OK:
        raise RuntimeError(f"{step}: {fmt_status(status)}")


def print_pose(prefix: str, pose_info) -> None:
    pos = pose_info.pose.position
    ori = pose_info.pose.orientation
    print(
        f"{prefix}: localized={pose_info.is_localization} "
        f"position=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
        f"orientation=({ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f})"
    )


def infer_map_path(map_name: str) -> str:
    return f"/home/eame/cust_para/maps/{map_name}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-ip", default=os.environ.get("MAGICBOT_Y1_LOCAL_IP", "192.168.54.111"))
    parser.add_argument("--map-name", default=f"sdk_basic_{int(time.time())}")
    parser.add_argument("--mapping-wait", type=float, default=3.0, help="seconds to keep mapping running before save")
    parser.add_argument("--save-timeout-ms", type=int, default=60000)
    parser.add_argument("--save-settle-wait", type=float, default=30.0, help="seconds to poll for map registration after save")
    parser.add_argument("--nav-x", type=float, default=0.3)
    parser.add_argument("--nav-y", type=float, default=0.0)
    parser.add_argument("--nav-yaw", type=float, default=0.0)
    parser.add_argument("--nav-wait", type=float, default=10.0, help="seconds to observe nav status before cancel")
    args = parser.parse_args()

    robot = magicbot.MagicRobot()
    slam = None
    map_path = None
    nav_started = False

    print(f"model: {magicbot.get_robot_model()}")
    print(f"sdk_version: {robot.get_sdk_version()}")
    print(f"local_ip: {args.local_ip}")
    print(f"map_name: {args.map_name}")

    try:
        if not robot.initialize(args.local_ip):
            print("robot.initialize failed", file=sys.stderr)
            return 1

        status = robot.connect()
        print(f"robot.connect: {fmt_status(status)}")
        require_ok(status, "robot.connect")

        slam = robot.get_slam_nav_controller()
        if not slam.initialize():
            print("slam.initialize failed", file=sys.stderr)
            return 2

        status = slam.activate_slam_mode(magicbot.SlamMode.MAPPING)
        print(f"activate_slam_mode(MAPPING): {fmt_status(status)}")
        require_ok(status, "activate_slam_mode(MAPPING)")

        status = slam.start_mapping()
        print(f"start_mapping: {fmt_status(status)}")
        require_ok(status, "start_mapping")

        if args.mapping_wait > 0:
            print(f"mapping_wait: sleeping {args.mapping_wait:.1f}s before save")
            time.sleep(args.mapping_wait)

        status = slam.save_map(args.map_name, args.save_timeout_ms)
        print(f"save_map({args.map_name}): {fmt_status(status)}")
        if status.code != magicbot.ErrorCode.OK:
            print("save_map did not return OK, continue polling map registration")

        deadline = time.time() + max(0.0, args.save_settle_wait)
        while time.time() < deadline:
            status, map_paths = slam.get_map_path(args.map_name, 10000)
            print(f"get_map_path({args.map_name}): {fmt_status(status)}")
            if status.code == magicbot.ErrorCode.OK and map_paths:
                map_path = map_paths[0]
                print(f"map_path: {map_path}")
                break
            if "deprecated" in status.message.lower():
                map_path = infer_map_path(args.map_name)
                print(f"get_map_path deprecated, fallback map_path: {map_path}")
                break
            time.sleep(2.0)

        if not map_path:
            raise RuntimeError(f"map '{args.map_name}' was not registered within {args.save_settle_wait:.1f}s")

        status = slam.activate_slam_mode(magicbot.SlamMode.LOCALIZATION, map_path)
        print(f"activate_slam_mode(LOCALIZATION): {fmt_status(status)}")
        require_ok(status, "activate_slam_mode(LOCALIZATION)")

        time.sleep(2.0)
        status, pose_info = slam.get_current_localization_info()
        print(f"get_current_localization_info: {fmt_status(status)}")
        require_ok(status, "get_current_localization_info")
        print_pose("current_pose", pose_info)

        status = slam.activate_nav_mode(magicbot.NavMode.GRID_MAP, map_path)
        print(f"activate_nav_mode(GRID_MAP): {fmt_status(status)}")
        require_ok(status, "activate_nav_mode(GRID_MAP)")

        target = magicbot.NavTarget()
        target.id = 1
        target.frame_id = "map"
        target.goal.position = [args.nav_x, args.nav_y, 0.0]
        target.goal.orientation = [0.0, 0.0, args.nav_yaw]
        status = slam.set_nav_target(target, 10000)
        print(f"set_nav_target(({args.nav_x:.3f}, {args.nav_y:.3f}, {args.nav_yaw:.3f})): {fmt_status(status)}")
        require_ok(status, "set_nav_target")
        nav_started = True

        deadline = time.time() + max(0.0, args.nav_wait)
        while time.time() < deadline:
            status, nav_status = slam.get_nav_task_status()
            print(f"get_nav_task_status: {fmt_status(status)}")
            if status.code == magicbot.ErrorCode.OK:
                print(
                    f"nav_status: id={nav_status.id} status={nav_status.status} "
                    f"error_code={nav_status.error_code} error_desc={nav_status.error_desc}"
                )
                if nav_status.status in (
                    magicbot.NavStatusType.END_SUCCESS,
                    magicbot.NavStatusType.END_FAILED,
                    magicbot.NavStatusType.CANCEL,
                ):
                    break
            else:
                print("nav_status detail unavailable yet")
            time.sleep(1.0)

        status, pose_info = slam.get_current_localization_info()
        print(f"final get_current_localization_info: {fmt_status(status)}")
        if status.code == magicbot.ErrorCode.OK:
            print_pose("final_pose", pose_info)

        return 0
    finally:
        if slam is not None and nav_started:
            try:
                status = slam.cancel_nav_task()
                print(f"cancel_nav_task: {fmt_status(status)}")
            except Exception as exc:
                print(f"cancel_nav_task exception: {exc}")
        try:
            if slam is not None:
                slam.shutdown()
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
