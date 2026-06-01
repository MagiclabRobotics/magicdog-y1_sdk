
add_executable(monitor_example monitor_example.cpp)

target_link_libraries(
  monitor_example
  PRIVATE magicdog_y1::sdk)
