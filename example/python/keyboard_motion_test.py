#!/usr/bin/env python3.11

import argparse
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import magicdog_y1_python as magicbot


def getch() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def fmt_status(status) -> str:
    return f"{status.code} {status.message}"


def print_help(axis_scale: float, turn_scale: float, pulse_s: float) -> None:
    print("Keyboard motion test")
    print(f"axis_scale={axis_scale:.2f} turn_scale={turn_scale:.2f} pulse_s={pulse_s:.2f}")
    print("  0: passive/down")
    print("  1: recovery stand")
    print("  2: terrain walk")
    print("  3: load walk")
    print("  4: crouch")
    print("  w/s: forward/backward hold")
    print("  a/d: strafe left/right hold")
    print("  q/e: rotate left/right hold")
    print("  space/x: stop")
    print("  h/?: help")
    print("  esc: exit")


def getch_with_timeout(timeout_s: float) -> str | None:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        readable, _, _ = select.select([fd], [], [], timeout_s)
        if not readable:
            return None
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-ip", default=os.environ.get("MAGICBOT_Y1_LOCAL_IP", "192.168.54.111"))
    parser.add_argument("--axis-scale", type=float, default=0.20)
    parser.add_argument("--turn-scale", type=float, default=0.20)
    parser.add_argument("--pulse-seconds", type=float, default=0.80)
    parser.add_argument("--hold-rate-hz", type=float, default=20.0)
    parser.add_argument("--release-timeout", type=float, default=0.20)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()

    running = True
    robot = magicbot.MagicRobot()
    hold_event = threading.Event()
    motion_thread = None
    command_lock = threading.Lock()
    active_command = (0.0, 0.0, 0.0, 0.0)
    command_deadline = 0.0
    current_gait = None

    def stop_motion() -> None:
        nonlocal active_command, command_deadline
        with command_lock:
            active_command = (0.0, 0.0, 0.0, 0.0)
            command_deadline = 0.0
        command = magicbot.JoystickCommand()
        command.left_x_axis = 0.0
        command.left_y_axis = 0.0
        command.right_x_axis = 0.0
        command.right_y_axis = 0.0
        robot.get_high_level_motion_controller().send_joystick_command(command)

    def run_hold() -> None:
        period_s = 1.0 / max(args.hold_rate_hz, 1.0)
        while running and not hold_event.is_set():
            with command_lock:
                left_x, left_y, right_x, right_y = active_command
                deadline = command_deadline
            now = time.time()
            if now > deadline:
                if any(abs(v) > 1e-6 for v in (left_x, left_y, right_x, right_y)):
                    stop_motion()
                time.sleep(period_s)
                continue
            command = magicbot.JoystickCommand()
            command.left_x_axis = left_x
            command.left_y_axis = left_y
            command.right_x_axis = right_x
            command.right_y_axis = right_y
            robot.get_high_level_motion_controller().send_joystick_command(command)
            time.sleep(period_s)
        stop_motion()

    def join_motion_thread() -> None:
        nonlocal motion_thread
        if motion_thread is not None:
            motion_thread.join(timeout=2.0)
            motion_thread = None

    def ensure_motion_thread() -> None:
        nonlocal motion_thread
        if motion_thread is not None and motion_thread.is_alive():
            return
        hold_event.clear()
        motion_thread = threading.Thread(target=run_hold, daemon=True)
        motion_thread.start()

    def set_active_command(left_x: float, left_y: float, right_x: float, right_y: float) -> None:
        with command_lock:
            nonlocal active_command, command_deadline
            active_command = (left_x, left_y, right_x, right_y)
            command_deadline = time.time() + max(args.release_timeout, 0.05)
        ensure_motion_thread()

    def set_motion(motion_id) -> None:
        nonlocal current_gait
        status = robot.get_high_level_motion_controller().set_motion_id(motion_id, 10000)
        print(f"set_motion_id: {fmt_status(status)}")
        if status.code == magicbot.ErrorCode.OK:
            current_gait = motion_id

    def set_walk(walk_gait_id) -> None:
        nonlocal current_gait
        status = robot.get_high_level_motion_controller().set_walk_gait_id(walk_gait_id, 10000)
        print(f"set_walk_gait_id: {fmt_status(status)}")
        if status.code == magicbot.ErrorCode.OK:
            current_gait = walk_gait_id

    def ensure_walk_ready() -> bool:
        nonlocal current_gait
        if current_gait in (magicbot.WalkGaitId.WALK_GAIT_TERRAIN, magicbot.WalkGaitId.WALK_GAIT_LOAD):
            return True
        status = robot.get_high_level_motion_controller().set_walk_gait_id(
            magicbot.WalkGaitId.WALK_GAIT_TERRAIN, 10000
        )
        print(f"auto set_walk_gait_id: {fmt_status(status)}")
        if status.code == magicbot.ErrorCode.OK:
            current_gait = magicbot.WalkGaitId.WALK_GAIT_TERRAIN
        return status.code == magicbot.ErrorCode.OK

    def handle_signal(signum, _frame) -> None:
        nonlocal running
        print(f"\nreceived signal {signum}, stopping")
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(f"model: {magicbot.get_robot_model()}")
    print(f"sdk_version: {robot.get_sdk_version()}")
    print(f"local_ip: {args.local_ip}")

    try:
        if not robot.initialize(args.local_ip):
            print("robot.initialize failed", file=sys.stderr)
            return 1

        status = robot.connect()
        print(f"robot.connect: {fmt_status(status)}")
        if status.code != magicbot.ErrorCode.OK:
            return 2

        status = robot.set_motion_control_level(magicbot.ControllerLevel.HighLevel)
        print(f"set_motion_control_level(HighLevel): {fmt_status(status)}")
        if status.code != magicbot.ErrorCode.OK:
            return 3

        controller = robot.get_high_level_motion_controller()
        if not controller.initialize():
            print("high_level_controller.initialize failed", file=sys.stderr)
            return 4

        status = controller.set_robot_mode(magicbot.RobotMode.ROBOT_MODE_REMOTE, 10000)
        print(f"set_robot_mode(REMOTE): {fmt_status(status)}")
        if status.code != magicbot.ErrorCode.OK:
            return 5

        if hasattr(controller, "get_walk_gait_id"):
            status, current_gait = controller.get_walk_gait_id()
            print(f"initial_walk_gait: {fmt_status(status)} {current_gait}")
            if status.code != magicbot.ErrorCode.OK:
                current_gait = None
        elif hasattr(controller, "get_gait"):
            status, current_gait = controller.get_gait()
            print(f"initial_gait: {fmt_status(status)} {current_gait}")
            if status.code != magicbot.ErrorCode.OK:
                current_gait = None
        else:
            current_gait = None
            print("initial_gait: unavailable")

        stop_motion()

        if args.self_check:
            print("self-check passed")
            return 0

        print_help(args.axis_scale, args.turn_scale, args.pulse_seconds)

        while running:
            key = getch_with_timeout(1.0 / max(args.hold_rate_hz, 1.0))
            if key is None:
                continue
            if key == "\x1b":
                break
            if key in ("h", "H", "?"):
                print_help(args.axis_scale, args.turn_scale, args.pulse_seconds)
            elif key == "0":
                set_motion(magicbot.MotionId.MOTION_PASSIVE)
            elif key == "1":
                set_motion(magicbot.MotionId.MOTION_RECOVER_STAND)
            elif key == "2":
                set_walk(magicbot.WalkGaitId.WALK_GAIT_TERRAIN)
            elif key == "3":
                set_walk(magicbot.WalkGaitId.WALK_GAIT_LOAD)
            elif key == "4":
                set_motion(magicbot.MotionId.MOTION_CROUCH)
            elif key in ("w", "W"):
                if not ensure_walk_ready():
                    continue
                set_active_command(0.0, args.axis_scale, 0.0, 0.0)
            elif key in ("s", "S"):
                if not ensure_walk_ready():
                    continue
                set_active_command(0.0, -args.axis_scale, 0.0, 0.0)
            elif key in ("a", "A"):
                if not ensure_walk_ready():
                    continue
                set_active_command(-args.axis_scale, 0.0, 0.0, 0.0)
            elif key in ("d", "D"):
                if not ensure_walk_ready():
                    continue
                set_active_command(args.axis_scale, 0.0, 0.0, 0.0)
            elif key in ("q", "Q"):
                if not ensure_walk_ready():
                    continue
                set_active_command(0.0, 0.0, -args.turn_scale, 0.0)
            elif key in ("e", "E"):
                if not ensure_walk_ready():
                    continue
                set_active_command(0.0, 0.0, args.turn_scale, 0.0)
            elif key in (" ", "x", "X"):
                print("stop")
                hold_event.set()
                join_motion_thread()
                stop_motion()
    finally:
        try:
            hold_event.set()
            join_motion_thread()
        except Exception:
            pass
        try:
            stop_motion()
        except Exception:
            pass
        try:
            robot.get_high_level_motion_controller().shutdown()
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

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
