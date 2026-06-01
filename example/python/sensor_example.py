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
    logging.info("Sensor example")
    logging.info("  1/2 open/close XT32 lidar")
    logging.info("  3/4 open/close front RGBD")
    logging.info("  5/6 open/close rear RGBD")
    logging.info("  7/8 open/close front fisheye")
    logging.info("  9/0 open/close rear fisheye")
    logging.info("  i/p toggle lidar IMU / point cloud")
    logging.info("  c/d/C toggle front RGBD color/depth/camera_info")
    logging.info("  v/b/V toggle rear RGBD color/depth/camera_info")
    logging.info("  f/F toggle front fisheye image/camera_info")
    logging.info("  r/R toggle rear fisheye image/camera_info")
    logging.info("  s show status, ? help, ESC exit")


class SensorManager:
    def __init__(self, sensor_controller):
        self.ctrl = sensor_controller
        self.opened = {
            "lidar": False,
            "front_rgbd": False,
            "rear_rgbd": False,
            "front_fisheye": False,
            "rear_fisheye": False,
        }
        self.subscribed = {
            "lidar_imu": False,
            "lidar_point_cloud": False,
            "front_rgbd_color": False,
            "front_rgbd_depth": False,
            "front_rgbd_info": False,
            "rear_rgbd_color": False,
            "rear_rgbd_depth": False,
            "rear_rgbd_info": False,
            "front_fisheye_image": False,
            "front_fisheye_info": False,
            "rear_fisheye_image": False,
            "rear_fisheye_info": False,
        }
        self.counters = {key: 0 for key in self.subscribed}

    def call_status(self, name, func):
        status = func()
        if status.code != magicbot.ErrorCode.OK:
            logging.error("%s failed: code=%s message=%s", name, status.code, status.message)
            return False
        logging.info("%s ok", name)
        return True

    def set_open(self, key, open_func, close_func, enable):
        if self.opened[key] == enable:
            logging.info("%s already %s", key, "open" if enable else "closed")
            return
        ok = self.call_status(("open " if enable else "close ") + key, open_func if enable else close_func)
        if ok:
            self.opened[key] = enable

    def toggle(self, key, subscribe_func, unsubscribe_func, callback):
        if self.subscribed[key]:
            unsubscribe_func()
            self.subscribed[key] = False
            logging.info("unsubscribe %s", key)
            return
        subscribe_func(callback)
        self.subscribed[key] = True
        logging.info("subscribe %s", key)

    def image_callback(self, key):
        def cb(image):
            self.counters[key] += 1
            if self.counters[key] % 15 == 0:
                logging.info("%s image size=%d resolution=%dx%d", key, len(image.data), image.width, image.height)

        return cb

    def camera_info_callback(self, key):
        def cb(info):
            self.counters[key] += 1
            if self.counters[key] % 30 == 0:
                logging.info("%s camera_info resolution=%dx%d", key, info.width, info.height)

        return cb

    def lidar_imu_callback(self, imu):
        self.counters["lidar_imu"] += 1
        if self.counters["lidar_imu"] % 100 == 0:
            logging.info("lidar imu timestamp=%d", imu.timestamp)

    def lidar_point_cloud_callback(self, point_cloud):
        self.counters["lidar_point_cloud"] += 1
        if self.counters["lidar_point_cloud"] % 10 == 0:
            logging.info("lidar point_cloud width=%d height=%d bytes=%d", point_cloud.width, point_cloud.height, len(point_cloud.data))

    def show_status(self):
        logging.info("opened=%s", self.opened)
        logging.info("subscribed=%s", self.subscribed)


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

        sensor_controller = robot.get_sensor_controller()
        if not sensor_controller.initialize():
            logging.error("Failed to initialize sensor controller")
            return -1

        mgr = SensorManager(sensor_controller)
        print_help()
        while running:
            choice = input("Enter command: ").strip()
            if choice in ("\x1b", "esc"):
                break
            if choice == "1":
                mgr.set_open("lidar", mgr.ctrl.open_lidar, mgr.ctrl.close_lidar, True)
            elif choice == "2":
                mgr.set_open("lidar", mgr.ctrl.open_lidar, mgr.ctrl.close_lidar, False)
            elif choice == "3":
                mgr.set_open("front_rgbd", mgr.ctrl.open_front_rgbd_camera, mgr.ctrl.close_front_rgbd_camera, True)
            elif choice == "4":
                mgr.set_open("front_rgbd", mgr.ctrl.open_front_rgbd_camera, mgr.ctrl.close_front_rgbd_camera, False)
            elif choice == "5":
                mgr.set_open("rear_rgbd", mgr.ctrl.open_rear_rgbd_camera, mgr.ctrl.close_rear_rgbd_camera, True)
            elif choice == "6":
                mgr.set_open("rear_rgbd", mgr.ctrl.open_rear_rgbd_camera, mgr.ctrl.close_rear_rgbd_camera, False)
            elif choice == "7":
                mgr.set_open("front_fisheye", mgr.ctrl.open_front_fisheye_camera, mgr.ctrl.close_front_fisheye_camera, True)
            elif choice == "8":
                mgr.set_open("front_fisheye", mgr.ctrl.open_front_fisheye_camera, mgr.ctrl.close_front_fisheye_camera, False)
            elif choice == "9":
                mgr.set_open("rear_fisheye", mgr.ctrl.open_rear_fisheye_camera, mgr.ctrl.close_rear_fisheye_camera, True)
            elif choice == "0":
                mgr.set_open("rear_fisheye", mgr.ctrl.open_rear_fisheye_camera, mgr.ctrl.close_rear_fisheye_camera, False)
            elif choice == "i":
                mgr.toggle("lidar_imu", mgr.ctrl.subscribe_lidar_imu, mgr.ctrl.unsubscribe_lidar_imu, mgr.lidar_imu_callback)
            elif choice == "p":
                mgr.toggle("lidar_point_cloud", mgr.ctrl.subscribe_lidar_point_cloud, mgr.ctrl.unsubscribe_lidar_point_cloud, mgr.lidar_point_cloud_callback)
            elif choice == "c":
                mgr.toggle("front_rgbd_color", mgr.ctrl.subscribe_front_rgbd_color_image, mgr.ctrl.unsubscribe_front_rgbd_color_image, mgr.image_callback("front_rgbd_color"))
            elif choice == "d":
                mgr.toggle("front_rgbd_depth", mgr.ctrl.subscribe_front_rgbd_depth_image, mgr.ctrl.unsubscribe_front_rgbd_depth_image, mgr.image_callback("front_rgbd_depth"))
            elif choice == "C":
                mgr.toggle("front_rgbd_info", mgr.ctrl.subscribe_front_rgbd_camera_info, mgr.ctrl.unsubscribe_front_rgbd_camera_info, mgr.camera_info_callback("front_rgbd_info"))
            elif choice == "v":
                mgr.toggle("rear_rgbd_color", mgr.ctrl.subscribe_rear_rgbd_color_image, mgr.ctrl.unsubscribe_rear_rgbd_color_image, mgr.image_callback("rear_rgbd_color"))
            elif choice == "b":
                mgr.toggle("rear_rgbd_depth", mgr.ctrl.subscribe_rear_rgbd_depth_image, mgr.ctrl.unsubscribe_rear_rgbd_depth_image, mgr.image_callback("rear_rgbd_depth"))
            elif choice == "V":
                mgr.toggle("rear_rgbd_info", mgr.ctrl.subscribe_rear_rgbd_camera_info, mgr.ctrl.unsubscribe_rear_rgbd_camera_info, mgr.camera_info_callback("rear_rgbd_info"))
            elif choice == "f":
                mgr.toggle("front_fisheye_image", mgr.ctrl.subscribe_front_fisheye_image, mgr.ctrl.unsubscribe_front_fisheye_image, mgr.image_callback("front_fisheye_image"))
            elif choice == "F":
                mgr.toggle("front_fisheye_info", mgr.ctrl.subscribe_front_fisheye_camera_info, mgr.ctrl.unsubscribe_front_fisheye_camera_info, mgr.camera_info_callback("front_fisheye_info"))
            elif choice == "r":
                mgr.toggle("rear_fisheye_image", mgr.ctrl.subscribe_rear_fisheye_image, mgr.ctrl.unsubscribe_rear_fisheye_image, mgr.image_callback("rear_fisheye_image"))
            elif choice == "R":
                mgr.toggle("rear_fisheye_info", mgr.ctrl.subscribe_rear_fisheye_camera_info, mgr.ctrl.unsubscribe_rear_fisheye_camera_info, mgr.camera_info_callback("rear_fisheye_info"))
            elif choice == "s":
                mgr.show_status()
            elif choice == "?":
                print_help()
            elif choice:
                logging.warning("Unknown command: %s", choice)
            time.sleep(0.01)

    except (EOFError, KeyboardInterrupt):
        pass
    except Exception as exc:
        logging.error("Example failed: %s", exc)
        return -1
    finally:
        if robot:
            try:
                robot.get_sensor_controller().shutdown()
                robot.disconnect()
                robot.shutdown()
            except Exception as exc:
                logging.error("Cleanup failed: %s", exc)

    return 0


if __name__ == "__main__":
    sys.exit(main())
