cmake_minimum_required(VERSION 3.8)
project(remote_rosbag_record)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# record — the recording server node (runs on the robot)
add_executable(record src/record.cpp)
target_compile_features(record PUBLIC cxx_std_17)
ament_target_dependencies(record
  rclcpp rosbag2_cpp rosbag2_storage rosbag2_transport std_msgs std_srvs)

# trigger — one-shot CLI to start/stop by service-name regex
add_executable(trigger src/trigger.cpp)
target_compile_features(trigger PUBLIC cxx_std_17)
ament_target_dependencies(trigger rclcpp std_srvs)

# joy_listener — gamepad-triggered start/stop
add_executable(joy_listener src/joy_listener.cpp)
target_compile_features(joy_listener PUBLIC cxx_std_17)
ament_target_dependencies(joy_listener rclcpp sensor_msgs std_srvs)

install(TARGETS record trigger joy_listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
