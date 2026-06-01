#pragma once

#include "magic_export.h"
#include "magic_type.h"

#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace magic::y1::motion {

class LowLevelMotionController;
using LowLevelMotionControllerPtr = std::unique_ptr<LowLevelMotionController>;

class HighLevelMotionController;
using HighLevelMotionControllerPtr = std::unique_ptr<HighLevelMotionController>;

/**
 * @brief Abstract base class that defines common interfaces for robot motion controllers.
 *
 * MotionControllerBase is the base class for all motion controllers, providing pure virtual function interfaces
 * for initializing and shutting down controllers. Derived classes need to implement these interfaces to meet specific control requirements.
 */
class MAGIC_EXPORT_API MotionControllerBase : public NonCopyable {
 public:
  /**
   * @brief Constructor.
   */
  MotionControllerBase() = default;

  /**
   * @brief Virtual destructor, ensures proper resource release in derived classes.
   */
  virtual ~MotionControllerBase() = default;

  /**
   * @brief Initialize the controller.
   * @return Returns true on successful initialization, false otherwise.
   */
  virtual bool Initialize() = 0;

  /**
   * @brief Shutdown the controller and release related resources.
   */
  virtual void Shutdown() = 0;

 protected:
  std::atomic_bool is_shutdown_{true};  // Mark whether initialized
};

/**
 * @class HighLevelMotionController
 * @brief High-level motion controller for semantic-level motion control of robots.
 *
 * This class inherits from MotionControllerBase and is mainly oriented towards high-level user interfaces, hiding low-level details.
 */
class MAGIC_EXPORT_API HighLevelMotionController final : public MotionControllerBase {
 public:
  /// Constructor, initializes internal state of high-level controller.
  HighLevelMotionController();

  /// Destructor, releases resources.
  virtual ~HighLevelMotionController();

  /**
   * @brief Initialize the controller, prepare high-level control functionality.
   * @return Whether initialization was successful.
   */
  virtual bool Initialize() override;

  /**
   * @brief Shutdown the controller and release related resources.
   */
  virtual void Shutdown() override;

  Status SetWalkGaitId(WalkGaitId walk_gait_id, int timeout_ms = 10000);
  Status GetWalkGaitId(WalkGaitId& walk_gait_id, int timeout_ms = 10000);
  Status SetMotionId(MotionId motion_id, int timeout_ms = 10000);
  Status GetMotionId(MotionId& motion_id, int timeout_ms = 10000);
  Status SetRobotMode(RobotMode robot_mode, int timeout_ms = 10000);

  /**
   * @brief Send real-time joystick control commands. Recommended sending frequency is 20Hz.
   * @param joy_command Control command containing left and right joystick coordinates.
   * @return Execution status.
   */
  Status SendJoyStickCommand(JoystickCommand& joy_command);

};

/**
 * @class LowLevelMotionController
 * @brief Low-level motion controller for A2203 leg and body IMU interfaces.
 *
 * Oriented towards low-level developers or control systems, providing command sending and state reading interfaces for various body components.
 */
class MAGIC_EXPORT_API LowLevelMotionController final : public MotionControllerBase {
  // Message pointer type definitions (smart pointers for memory management)
  using JointStatePtr = std::shared_ptr<JointState>;  // Joint state message pointer
  using ImuPtr = std::shared_ptr<Imu>;                // IMU inertial measurement unit message pointer

  // Callback function type definitions for various joint data
  using LegJointStateCallback = std::function<void(const JointStatePtr)>;    // Leg joint state callback function type
  using BodyImuCallback = std::function<void(const ImuPtr)>;                 // Body IMU data callback

 public:
  /// Constructor, initializes low-level controller.
  LowLevelMotionController();

  /// Destructor, releases resources.
  virtual ~LowLevelMotionController();

  /**
   * @brief Initialize the controller, establish low-level motion control connection.
   * @return Whether initialization was successful.
   */
  virtual bool Initialize() override;

  /**
   * @brief Shutdown the controller and release low-level resources.
   */
  virtual void Shutdown() override;

  // === Leg Control ===

  /**
   * @brief Subscribe to leg joint state data
   * @param callback Callback function for processing received leg joint state data
   */
  void SubscribeLegState(LegJointStateCallback callback);

  /**
   * @brief Unsubscribe from leg joint state data
   */
  void UnsubscribeLegState();

  /**
   * @brief Publish leg joint control command
   * @param command Leg joint control command containing target angle/velocity and other control information
   * @return Execution status.
   */
  Status PublishLegCommand(const JointCommand& command);

  /**
   * @brief Subscribe to body IMU data
   * @param callback Processing callback after receiving IMU data
   */
  void SubscribeBodyImu(const BodyImuCallback callback);

  /**
   * @brief Unsubscribe from body IMU data
   */
  void UnsubscribeBodyImu();
};

}  // namespace magic::y1::motion
