#include "magic_robot.h"
#include "magic_sdk_version.h"

#include <termios.h>
#include <unistd.h>
#include <csignal>

#include <iostream>

using namespace magic::y1;

magic::y1::MagicRobot robot;

void signalHandler(int signum) {
  std::cout << "Interrupt signal (" << signum << ") received.\n";

  robot.Shutdown();
  // Exit process
  exit(signum);
}

void print_help() {
  std::cout << "Key Function Demo Program\n\n";
  std::cout << "High-Level Motion Control Function Description:\n";
  std::cout << "  1        Function 1: Recovery stand\n";
  std::cout << "  2        Function 2: Terrain walk\n";
  std::cout << "  3        Function 3: Load walk\n";
  std::cout << "  4        Function 4: Crouch\n";
  std::cout << "  w        Function w: Move forward\n";
  std::cout << "  a        Function a: Move left\n";
  std::cout << "  s        Function s: Move backward\n";
  std::cout << "  d        Function d: Move right\n";
  std::cout << "  x        Function x: Stop moving\n";
  std::cout << "  t        Function t: Turn left\n";
  std::cout << "  g        Function g: Turn right\n";
  std::cout << "\n";
  std::cout << "  ?        Function ?: Print help\n";
  std::cout << "  ESC      Exit program\n";
}

int getch() {
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);  // Get current terminal settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);  // Disable buffering and echo
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();                           // Read key press
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore settings
  return ch;
}

void RecoveryStand() {
  // Get high-level motion controller
  auto& controller = robot.GetHighLevelMotionController();

  // Set gait
  auto status = controller.SetMotionId(MotionId::MOTION_RECOVER_STAND);
  if (status.code != ErrorCode::OK) {
    std::cerr << "set robot motion failed"
              << ", code: " << status.code
              << ", message: " << status.message << std::endl;
    return;
  }
  std::cout << "robot motion set to MOTION_RECOVER_STAND successfully." << std::endl;
}

void TerrainWalk() {
  // Get high-level motion controller
  auto& controller = robot.GetHighLevelMotionController();

  // Set walk gait
  auto status = controller.SetWalkGaitId(WalkGaitId::WALK_GAIT_TERRAIN);
  if (status.code != ErrorCode::OK) {
    std::cerr << "set robot walk gait failed"
              << ", code: " << status.code
              << ", message: " << status.message << std::endl;
    return;
  }
  std::cout << "robot walk gait set to WALK_GAIT_TERRAIN successfully." << std::endl;
}

void LoadWalk() {
  // Get high-level motion controller
  auto& controller = robot.GetHighLevelMotionController();

  // Set walk gait
  auto status = controller.SetWalkGaitId(WalkGaitId::WALK_GAIT_LOAD);
  if (status.code != ErrorCode::OK) {
    std::cerr << "set robot walk gait failed"
              << ", code: " << status.code
              << ", message: " << status.message << std::endl;
    return;
  }
  std::cout << "robot walk gait set to WALK_GAIT_LOAD successfully." << std::endl;
}

void Crouch() {
  // Get high-level motion controller
  auto& controller = robot.GetHighLevelMotionController();

  // Set motion id
  auto status = controller.SetMotionId(MotionId::MOTION_CROUCH);
  if (status.code != ErrorCode::OK) {
    std::cerr << "set robot motion failed"
              << ", code: " << status.code
              << ", message: " << status.message << std::endl;
    return;
  }
  std::cout << "robot motion set to MOTION_CROUCH successfully." << std::endl;
}

void JoyStickCommand(float left_x_axis,
                     float left_y_axis,
                     float right_x_axis,
                     float right_y_axis) {
  // Get high-level motion controller
  auto& controller = robot.GetHighLevelMotionController();

  JoystickCommand joy_command;
  joy_command.left_x_axis = left_x_axis;
  joy_command.left_y_axis = left_y_axis;
  joy_command.right_x_axis = right_x_axis;
  joy_command.right_y_axis = right_y_axis;
  controller.SendJoyStickCommand(joy_command);
}

int main(int argc, char* argv[]) {
  // Bind SIGINT (Ctrl+C)
  signal(SIGINT, signalHandler);

  std::cout << "SDK Version: " << SDK_VERSION_STRING << std::endl;

  print_help();

  std::string local_ip = "192.168.54.111";
  // Configure local IP address for direct ethernet connection to robot and initialize SDK
  if (!robot.Initialize(local_ip)) {
    std::cerr << "robot sdk initialize failed." << std::endl;
    robot.Shutdown();
    return -1;
  }

  // Connect to robot
  auto status = robot.Connect();
  if (status.code != ErrorCode::OK) {
    std::cerr << "connect robot failed"
              << ", code: " << status.code
              << ", message: " << status.message << std::endl;
    robot.Shutdown();
    return -1;
  }

  // Switch motion controller to high-level controller, default is high-level controller
  status = robot.SetMotionControlLevel(ControllerLevel::HighLevel);
  if (status.code != ErrorCode::OK) {
    std::cerr << "switch robot motion control level failed"
              << ", code: " << status.code
              << ", message: " << status.message << std::endl;
    robot.Shutdown();
    return -1;
  }

  std::cout << "Press any key to continue (ESC to exit)..."
            << std::endl;

  // Wait for user input
  while (1) {
    int key = getch();
    if (key == 27)
      break;  // ESC key ASCII code is 27

    std::cout << "Key ASCII: " << key << ", Character: " << static_cast<char>(key) << std::endl;
    switch (key) {
      case '1': {
        RecoveryStand();
        break;
      }
      case '2': {
        TerrainWalk();
        break;
      }
      case '3': {
        LoadWalk();
        break;
      }
      case '4': {
        Crouch();
        break;
      }
      case 'W':
      case 'w': {
        JoyStickCommand(0.0, 1.0, 0.0, 0.0);  // Move forward
        break;
      }
      case 'A':
      case 'a': {
        JoyStickCommand(-1.0, 0.0, 0.0, 0.0);  // Move left
        break;
      }
      case 'S':
      case 's': {
        JoyStickCommand(0.0, -1.0, 0.0, 0.0);  // Move backward
        break;
      }
      case 'D':
      case 'd': {
        JoyStickCommand(1.0, 0.0, 0.0, 0.0);  // Move right
        break;
      }
      case 'X':
      case 'x': {
        JoyStickCommand(0.0, 0.0, 0.0, 0.0);  // Stop
        break;
      }
      case 'T':
      case 't': {
        JoyStickCommand(0.0, 0.0, -1.0, 1.0);  // Turn left
        break;
      }
      case 'G':
      case 'g': {
        JoyStickCommand(0.0, 0.0, 1.0, 1.0);  // Turn right
        break;
      }
      case '?': {
        print_help();
        break;
      }
      default:
        std::cout << "Unknown key: " << key << std::endl;
        break;
    }
  }

  // Disconnect from robot
  status = robot.Disconnect();
  if (status.code != ErrorCode::OK) {
    std::cerr << "disconnect robot failed"
              << ", code: " << status.code
              << ", message: " << status.message << std::endl;
    robot.Shutdown();
    return -1;
  }

  robot.Shutdown();

  return 0;
}
