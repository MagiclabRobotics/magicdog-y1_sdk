#!/usr/bin/env python3.11

import os
import sys

import magicdog_y1_python as magicbot


def main() -> int:
    robot = magicbot.MagicRobot()
    local_ip = os.environ.get("MAGICBOT_Y1_LOCAL_IP", "192.168.54.111")

    print(f"model: {magicbot.get_robot_model()}")
    print(f"sdk_version: {robot.get_sdk_version()}")

    try:
        if not robot.initialize(local_ip):
            print(f"initialize failed: local_ip={local_ip}", file=sys.stderr)
            return 1

        status = robot.connect()
        print(f"connect: {status.code} {status.message}")
        if status.code != magicbot.ErrorCode.OK:
            return 2

        monitor = robot.get_state_monitor()
        if not monitor.initialize():
            print("state monitor initialize failed", file=sys.stderr)
            return 3

        status, state = monitor.get_current_state()
        print(f"get_current_state: {status.code} {status.message}")
        if status.code != magicbot.ErrorCode.OK:
            return 4

        print(f"battery_health: {state.bms_data.battery_health}")
        print(f"battery_percentage: {state.bms_data.battery_percentage}")
        print(f"battery_state: {state.bms_data.battery_state}")
        print(f"power_supply_status: {state.bms_data.power_supply_status}")
        print(f"fault_count: {len(state.faults)}")
        for fault in state.faults:
            print(f"fault: code={fault.error_code} message={fault.error_message}")

        return 0
    finally:
        try:
            robot.get_state_monitor().shutdown()
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
