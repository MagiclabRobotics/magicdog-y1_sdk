#!/usr/bin/env python3

import argparse
import time
from dataclasses import dataclass
from typing import Callable

import magicdog_y1_python as magicbot


@dataclass
class Result:
    name: str
    ok: bool
    detail: str = ""


def fmt_status(status) -> str:
    return f"{status.code} {status.message}"


def run_step(results: list[Result], name: str, func: Callable[[], str | None]) -> None:
    try:
        detail = func() or ""
        results.append(Result(name, True, detail))
        print(f"[OK]   {name} {detail}")
    except Exception as exc:
        results.append(Result(name, False, str(exc)))
        print(f"[FAIL] {name} {exc}")


def status_detail(status) -> str:
    require_ok(status, "status")
    return fmt_status(status)


def require_ok(status, name: str):
    if status.code != magicbot.ErrorCode.OK:
        raise RuntimeError(f"{name}: {fmt_status(status)}")


def wait_for_callback(subscribe, unsubscribe, label: str, timeout_s: float, validator: Callable[[object], None] | None = None):
    count = 0
    sample = {}

    def cb(msg):
        nonlocal count, sample
        count += 1
        if count == 1:
            sample = summarize_msg(msg)
            if validator is not None:
                validator(msg)

    subscribe(cb)
    deadline = time.time() + timeout_s
    try:
        while time.time() < deadline and count == 0:
            time.sleep(0.05)
    finally:
        unsubscribe()
    return f"{label}: callbacks={count} sample={sample}"


def summarize_msg(msg):
    fields = {}
    for name in ("timestamp", "width", "height", "encoding", "data_length", "format", "id", "status", "error_code"):
        if hasattr(msg, name):
            fields[name] = getattr(msg, name)
    if hasattr(msg, "data"):
        fields["data_len"] = len(msg.data)
    if hasattr(msg, "raw_data"):
        fields["raw_data_len"] = len(msg.raw_data)
    if hasattr(msg, "joints"):
        fields["joint_count"] = len(msg.joints)
    return fields


def require_imu_sample(msg):
    if not hasattr(msg, "timestamp"):
        raise RuntimeError("imu sample missing timestamp")
    if not hasattr(msg, "orientation"):
        raise RuntimeError("imu sample missing orientation")


def require_leg_state_sample(msg):
    if not hasattr(msg, "timestamp"):
        raise RuntimeError("leg state sample missing timestamp")
    if not hasattr(msg, "joints"):
        raise RuntimeError("leg state sample missing joints")
    if len(msg.joints) == 0:
        raise RuntimeError("leg state sample has no joints")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-ip", default="192.168.54.111")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--include-motion", action="store_true", help="also send safe high-level read-only motion calls")
    parser.add_argument("--include-streams", action="store_true", help="open streams and wait briefly for callbacks")
    args = parser.parse_args()

    results: list[Result] = []
    robot = magicbot.MagicRobot()

    print(f"model={magicbot.get_robot_model()}")
    print(f"sdk_version={robot.get_sdk_version()}")
    print(f"local_ip={args.local_ip}")

    try:
        run_step(results, "robot.initialize", lambda: f"ok={robot.initialize(args.local_ip)}")
        run_step(results, "robot.connect", lambda: status_detail(robot.connect()))

        state_monitor = robot.get_state_monitor()
        run_step(results, "state_monitor.initialize", lambda: f"ok={state_monitor.initialize()}")

        def get_current_state():
            status, state = state_monitor.get_current_state()
            require_ok(status, "get_current_state")
            return (
                f"battery={state.bms_data.battery_percentage} "
                f"battery_state={state.bms_data.battery_state} faults={len(state.faults)}"
            )

        run_step(results, "state_monitor.get_current_state", get_current_state)

        audio = robot.get_audio_controller()
        run_step(results, "audio.initialize", lambda: f"ok={audio.initialize()}")

        def get_volume():
            status, volume = audio.get_volume()
            require_ok(status, "audio.get_volume")
            return f"{fmt_status(status)} volume={volume}"

        run_step(results, "audio.get_volume", get_volume)
        if args.include_streams:
            run_step(results, "audio.open_audio_stream", lambda: status_detail(audio.open_audio_stream()))
            run_step(
                results,
                "audio.subscribe_origin_audio_stream",
                lambda: wait_for_callback(audio.subscribe_origin_audio_stream, audio.unsubscribe_origin_audio_stream, "origin_audio", args.timeout),
            )
            run_step(
                results,
                "audio.subscribe_bf_audio_stream",
                lambda: wait_for_callback(audio.subscribe_bf_audio_stream, audio.unsubscribe_bf_audio_stream, "bf_audio", args.timeout),
            )
            run_step(results, "audio.close_audio_stream", lambda: status_detail(audio.close_audio_stream()))

        sensor = robot.get_sensor_controller()
        run_step(results, "sensor.initialize", lambda: f"ok={sensor.initialize()}")
        if args.include_streams:
            sensor_streams = [
                ("lidar", sensor.open_lidar, sensor.close_lidar),
                ("front_rgbd", sensor.open_front_rgbd_camera, sensor.close_front_rgbd_camera),
                ("rear_rgbd", sensor.open_rear_rgbd_camera, sensor.close_rear_rgbd_camera),
                ("front_fisheye", sensor.open_front_fisheye_camera, sensor.close_front_fisheye_camera),
                ("rear_fisheye", sensor.open_rear_fisheye_camera, sensor.close_rear_fisheye_camera),
            ]
            for name, open_fn, close_fn in sensor_streams:
                run_step(results, f"sensor.open_{name}", lambda open_fn=open_fn: status_detail(open_fn()))

            callbacks = [
                ("lidar_imu", sensor.subscribe_lidar_imu, sensor.unsubscribe_lidar_imu),
                ("lidar_point_cloud", sensor.subscribe_lidar_point_cloud, sensor.unsubscribe_lidar_point_cloud),
                ("front_rgbd_color", sensor.subscribe_front_rgbd_color_image, sensor.unsubscribe_front_rgbd_color_image),
                ("front_rgbd_depth", sensor.subscribe_front_rgbd_depth_image, sensor.unsubscribe_front_rgbd_depth_image),
                ("front_rgbd_info", sensor.subscribe_front_rgbd_camera_info, sensor.unsubscribe_front_rgbd_camera_info),
                ("rear_rgbd_color", sensor.subscribe_rear_rgbd_color_image, sensor.unsubscribe_rear_rgbd_color_image),
                ("rear_rgbd_depth", sensor.subscribe_rear_rgbd_depth_image, sensor.unsubscribe_rear_rgbd_depth_image),
                ("rear_rgbd_info", sensor.subscribe_rear_rgbd_camera_info, sensor.unsubscribe_rear_rgbd_camera_info),
                ("front_fisheye_image", sensor.subscribe_front_fisheye_image, sensor.unsubscribe_front_fisheye_image),
                ("front_fisheye_info", sensor.subscribe_front_fisheye_camera_info, sensor.unsubscribe_front_fisheye_camera_info),
                ("rear_fisheye_image", sensor.subscribe_rear_fisheye_image, sensor.unsubscribe_rear_fisheye_image),
                ("rear_fisheye_info", sensor.subscribe_rear_fisheye_camera_info, sensor.unsubscribe_rear_fisheye_camera_info),
            ]
            for name, sub, unsub in callbacks:
                run_step(results, f"sensor.subscribe_{name}", lambda sub=sub, unsub=unsub, name=name: wait_for_callback(sub, unsub, name, args.timeout))

            for name, _open_fn, close_fn in reversed(sensor_streams):
                run_step(results, f"sensor.close_{name}", lambda close_fn=close_fn: status_detail(close_fn()))

        slam = robot.get_slam_nav_controller()
        run_step(results, "slam.initialize", lambda: f"ok={slam.initialize()}")
        run_step(results, "slam.get_current_localization_info", lambda: status_detail(slam.get_current_localization_info()[0]))
        run_step(results, "slam.get_nav_task_status", lambda: status_detail(slam.get_nav_task_status()[0]))

        def get_all_map_info():
            status, info = slam.get_all_map_info(3000)
            require_ok(status, "slam.get_all_map_info")
            return f"{fmt_status(status)} maps={len(info.map_infos)}"

        run_step(results, "slam.get_all_map_info", get_all_map_info)
        run_step(results, "slam.get_point_cloud_map", lambda: status_detail(slam.get_point_cloud_map(3000)[0]))
        if args.include_streams:
            run_step(results, "slam.open_odometry_stream", lambda: status_detail(slam.open_odometry_stream()))
            run_step(results, "slam.subscribe_odometry", lambda: wait_for_callback(slam.subscribe_odometry, slam.unsubscribe_odometry, "odometry", args.timeout))
            run_step(results, "slam.close_odometry_stream", lambda: status_detail(slam.close_odometry_stream()))

        high = robot.get_high_level_motion_controller()
        low = robot.get_low_level_motion_controller()
        run_step(results, "high_level.initialize", lambda: f"ok={high.initialize()}")
        run_step(results, "low_level.initialize", lambda: f"ok={low.initialize()}")
        if args.include_motion:
            def get_walk_gait_id():
                status, walk_gait_id = high.get_walk_gait_id()
                require_ok(status, "high_level.get_walk_gait_id")
                return f"{fmt_status(status)} walk_gait_id={walk_gait_id}"

            run_step(results, "high_level.get_walk_gait_id", get_walk_gait_id)
        if args.include_streams:
            run_step(
                results,
                "low_level.subscribe_body_imu",
                lambda: wait_for_callback(
                    low.subscribe_body_imu,
                    low.unsubscribe_body_imu,
                    "body_imu",
                    args.timeout,
                    require_imu_sample,
                ),
            )
            run_step(
                results,
                "low_level.subscribe_leg_state",
                lambda: wait_for_callback(
                    low.subscribe_leg_state,
                    low.unsubscribe_leg_state,
                    "leg_state",
                    args.timeout,
                    require_leg_state_sample,
                ),
            )

    finally:
        for cleanup in (
            lambda: robot.get_low_level_motion_controller().shutdown(),
            lambda: robot.get_high_level_motion_controller().shutdown(),
            lambda: robot.get_slam_nav_controller().shutdown(),
            lambda: robot.get_sensor_controller().shutdown(),
            lambda: robot.get_audio_controller().shutdown(),
            lambda: robot.get_state_monitor().shutdown(),
            robot.disconnect,
            robot.shutdown,
        ):
            try:
                cleanup()
            except Exception:
                pass

    failed = [r for r in results if not r.ok]
    print(f"\nsummary: total={len(results)} passed={len(results) - len(failed)} failed={len(failed)}")
    for item in failed:
        print(f"failed: {item.name}: {item.detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
