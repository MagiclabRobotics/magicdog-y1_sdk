add_executable(navigation_example navigation_example.cpp)

target_link_libraries(
  navigation_example
  PRIVATE magicdog_y1::sdk)

add_executable(slam_example slam_example.cpp)

target_link_libraries(
  slam_example
  PRIVATE magicdog_y1::sdk)
