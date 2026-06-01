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


def signal_handler(signum, frame):
    global running
    logging.info("Received signal %s, exiting", signum)
    running = False


def print_help():
    logging.info("Audio example")
    logging.info("  1 get volume")
    logging.info("  2 set volume [0-100]")
    logging.info("  3 play TTS [text]")
    logging.info("  4 stop TTS")
    logging.info("  5 open audio stream")
    logging.info("  6 close audio stream")
    logging.info("  7 subscribe audio stream")
    logging.info("  8 unsubscribe audio stream")
    logging.info("  ? help")
    logging.info("  ESC exit")


def controller():
    return robot.get_audio_controller()


def get_volume():
    status, volume = controller().get_volume()
    if status.code != magicbot.ErrorCode.OK:
        logging.error("get_volume failed: code=%s message=%s", status.code, status.message)
        return
    logging.info("Volume: %s", volume)


def set_volume(value):
    status = controller().set_volume(int(value))
    if status.code != magicbot.ErrorCode.OK:
        logging.error("set_volume failed: code=%s message=%s", status.code, status.message)


def play_tts(text):
    tts = magicbot.TtsCommand()
    tts.id = "100000000001"
    tts.content = text
    tts.priority = magicbot.TtsPriority.HIGH
    tts.mode = magicbot.TtsMode.CLEARTOP
    status = controller().play(tts)
    if status.code != magicbot.ErrorCode.OK:
        logging.error("play TTS failed: code=%s message=%s", status.code, status.message)


def stop_tts():
    status = controller().stop()
    if status.code != magicbot.ErrorCode.OK:
        logging.error("stop TTS failed: code=%s message=%s", status.code, status.message)


def open_audio_stream():
    status = controller().open_audio_stream()
    if status.code != magicbot.ErrorCode.OK:
        logging.error("open_audio_stream failed: code=%s message=%s", status.code, status.message)


def close_audio_stream():
    status = controller().close_audio_stream()
    if status.code != magicbot.ErrorCode.OK:
        logging.error("close_audio_stream failed: code=%s message=%s", status.code, status.message)


def subscribe_audio_stream():
    origin_counter = 0
    bf_counter = 0

    def origin_callback(audio_stream):
        nonlocal origin_counter
        if origin_counter % 30 == 0:
            logging.info("Origin audio frame size=%d", audio_stream.data_length)
        origin_counter += 1

    def bf_callback(audio_stream):
        nonlocal bf_counter
        if bf_counter % 30 == 0:
            logging.info("BF audio frame size=%d", audio_stream.data_length)
        bf_counter += 1

    controller().subscribe_origin_audio_stream(origin_callback)
    controller().subscribe_bf_audio_stream(bf_callback)


def unsubscribe_audio_stream():
    controller().unsubscribe_bf_audio_stream()
    controller().unsubscribe_origin_audio_stream()


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

        if not controller().initialize():
            logging.error("Failed to initialize audio controller")
            return -1

        print_help()
        while running:
            line = input("Enter command: ").strip()
            if line in ("\x1b", "esc"):
                break
            if not line:
                time.sleep(0.01)
                continue
            parts = line.split(maxsplit=1)
            key = parts[0]
            arg = parts[1] if len(parts) > 1 else ""
            if key == "1":
                get_volume()
            elif key == "2":
                set_volume(arg or 50)
            elif key == "3":
                play_tts(arg or "hello")
            elif key == "4":
                stop_tts()
            elif key == "5":
                open_audio_stream()
            elif key == "6":
                close_audio_stream()
            elif key == "7":
                subscribe_audio_stream()
            elif key == "8":
                unsubscribe_audio_stream()
            elif key == "?":
                print_help()
            else:
                logging.warning("Unknown command: %s", key)

    except (EOFError, KeyboardInterrupt):
        pass
    except Exception as exc:
        logging.error("Example failed: %s", exc)
        return -1
    finally:
        if robot:
            try:
                robot.get_audio_controller().shutdown()
                robot.disconnect()
                robot.shutdown()
            except Exception as exc:
                logging.error("Cleanup failed: %s", exc)

    return 0


if __name__ == "__main__":
    sys.exit(main())
