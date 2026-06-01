#pragma once

#include "magic_export.h"
#include "magic_type.h"

#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace magic::y1::sensor {

class SensorController;
using SensorControllerPtr = std::unique_ptr<SensorController>;

/**
 * @class SensorController
 * @brief Sensor controller class that encapsulates initialization, on/off and data acquisition interfaces for various robot sensors.
 *
 * Supports acquiring IMU, point cloud, RGBD images, fisheye camera images and other information, providing unified access methods
 * for upper-level controllers or state fusion modules to call.
 */
class MAGIC_EXPORT_API SensorController final : public NonCopyable {
  // Message pointer type definitions (smart pointers for memory management)
  using PointCloudPtr = std::shared_ptr<PointCloud2>;                // Point cloud message pointer
  using ImuPtr = std::shared_ptr<Imu>;                               // IMU inertial measurement unit message pointer
  using ImagePtr = std::shared_ptr<Image>;                           // Image message pointer
  using CameraInfoPtr = std::shared_ptr<CameraInfo>;                 // Camera intrinsic parameter information pointer
  using FisheyeCameraPtr = std::shared_ptr<FisheyeCameraFrame>;  // Fisheye camera frame data pointer (spelling suggestion: change to Trinocular)

  // Callback function type definitions for various sensor data
  using LidarImuCallback = std::function<void(const ImuPtr)>;                    // Lidar IMU data callback
  using LidarPointCloudCallback = std::function<void(const PointCloudPtr)>;      // Lidar point cloud data callback
  using RgbdImageCallback = std::function<void(const ImagePtr)>;                 // RGBD image data callback
  using CameraInfoCallback = std::function<void(const CameraInfoPtr)>;           // RGBD camera intrinsic parameter callback
  using FisheyeImageCallback = std::function<void(const FisheyeCameraPtr)>;  // Fisheye camera image frame callback

 public:
  /// Constructor: Create SensorController instance, initialize internal state
  SensorController();

  /// Destructor: Release resources, close all sensors
  virtual ~SensorController();

  /**
   * @brief Initialize sensor controller, including resource allocation, driver loading, etc.
   * @return Returns true on successful initialization, false otherwise.
   */
  bool Initialize();

  /**
   * @brief Close all sensor connections and release resources.
   */
  void Shutdown();

  // === Lidar Control ===

  /**
   * @brief Open Lidar.
   * @return Operation status.
   */
  Status OpenLidar();

  /**
   * @brief Close Lidar.
   * @return Operation status.
   */
  Status CloseLidar();

  // === RGBD Camera Control ===

  /**
   * @brief Open front RGBD camera (including head and waist).
   * @return Operation status.
   */
  Status OpenFrontRgbdCamera();

  /**
   * @brief Close front RGBD camera.
   * @return Operation status.
   */
  Status CloseFrontRgbdCamera();

  /**
   * @brief Open rear RGBD camera.
   * @return Operation status.
   */
  Status OpenRearRgbdCamera();

  /**
   * @brief Close rear RGBD camera.
   * @return Operation status.
   */
  Status CloseRearRgbdCamera();

  // === Fisheye Camera Control ===

  /**
   * @brief Open fisheye camera.
   * @return Operation status.
   */
  Status OpenFrontFisheyeCamera();
  Status OpenRearFisheyeCamera();

  /**
   * @brief Close fisheye camera.
   * @return Operation status.
   */
  Status CloseFrontFisheyeCamera();
  Status CloseRearFisheyeCamera();

  // 订阅各类传感器数据的函数接口

  /**
   * @brief Subscribe to lidar IMU data
   * @param callback Processing callback after receiving lidar IMU data
   */
  void SubscribeLidarImu(const LidarImuCallback callback);

  /**
   * @brief Unsubscribe from lidar IMU data
   */
  void UnsubscribeLidarImu();

  /**
   * @brief Subscribe to lidar point cloud data
   * @param callback Processing callback after receiving point cloud data
   */
  void SubscribeLidarPointCloud(const LidarPointCloudCallback callback);

  /**
   * @brief Unsubscribe from lidar point cloud data
   */
  void UnsubscribeLidarPointCloud();

  /**
   * @brief Subscribe to front RGBD color image data
   * @param callback Processing callback after receiving image data
   */
  void SubscribeFrontRgbdColorImage(const RgbdImageCallback callback);

  /**
   * @brief Unsubscribe from front RGBD color image data
   */
  void UnsubscribeFrontRgbdColorImage();

  /**
   * @brief Subscribe to front RGBD depth image data
   * @param callback Processing callback after receiving depth image data
   */
  void SubscribeFrontRgbdDepthImage(const RgbdImageCallback callback);

  /**
   * @brief Unsubscribe from front RGBD depth image data
   */
  void UnsubscribeFrontRgbdDepthImage();

  /**
   * @brief Subscribe to front RGBD camera parameter data
   * @param callback Processing callback after receiving camera intrinsic parameter information
   */
  void SubscribeFrontRgbdCameraInfo(const CameraInfoCallback callback);

  /**
   * @brief Unsubscribe from front RGBD camera parameter data
   */
  void UnsubscribeFrontRgbdCameraInfo();

  /**
   * @brief Subscribe to rear RGBD color image data
   * @param callback Processing callback after receiving image data
   */
  void SubscribeRearRgbdColorImage(const RgbdImageCallback callback);

  /**
   * @brief Unsubscribe from rear RGBD color image data
   */
  void UnsubscribeRearRgbdColorImage();

  /**
   * @brief Subscribe to rear RGBD depth image data
   * @param callback Processing callback after receiving depth image data
   */
  void SubscribeRearRgbdDepthImage(const RgbdImageCallback callback);

  /**
   * @brief Unsubscribe from rear RGBD depth image data
   */
  void UnsubscribeRearRgbdDepthImage();

  /**
   * @brief Subscribe to rear RGBD camera parameter data
   * @param callback Processing callback after receiving camera intrinsic parameter information
   */
  void SubscribeRearRgbdCameraInfo(const CameraInfoCallback callback);

  /**
   * @brief Unsubscribe from rear RGBD camera parameter data
   */
  void UnsubscribeRearRgbdCameraInfo();

  /**
   * @brief Subscribe to fisheye camera image frame data
   * @param callback Processing callback after receiving fisheye camera data
   */
  void SubscribeFrontFisheyeImage(const FisheyeImageCallback callback);

  /**
   * @brief Unsubscribe from fisheye camera image frame data
   */
  void UnsubscribeFrontFisheyeImage();

  /**
   * @brief Subscribe to fisheye camera parameter data
   * @param callback Processing callback after receiving camera intrinsic parameter information
   */
  void SubscribeFrontFisheyeCameraInfo(const CameraInfoCallback callback);

  /**
   * @brief Unsubscribe from fisheye camera parameter data
   */
  void UnsubscribeFrontFisheyeCameraInfo();

  /**
   * @brief Subscribe to rear fisheye camera image frame data
   * @param callback Processing callback after receiving rear fisheye camera data
   */
  void SubscribeRearFisheyeImage(const FisheyeImageCallback callback);

  /**
   * @brief Unsubscribe from rear fisheye camera image frame data
   */
  void UnsubscribeRearFisheyeImage();

  /**
   * @brief Subscribe to rear fisheye camera parameter data
   * @param callback Processing callback after receiving camera intrinsic parameter information
   */
  void SubscribeRearFisheyeCameraInfo(const CameraInfoCallback callback);

  /**
   * @brief Unsubscribe from rear fisheye camera parameter data
   */
  void UnsubscribeRearFisheyeCameraInfo();

 private:
  std::atomic_bool is_shutdown_{true};  // Mark whether initialized
};

}  // namespace magic::y1::sensor
