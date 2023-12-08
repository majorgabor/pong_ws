cmake_minimum_required(VERSION 3.8)
project(pong_ros_server)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pong_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include our own headers
include_directories(include)

# Create a node
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/pong_logic.cpp
  src/pong_ros_server_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pong_msgs
  std_msgs
)

# Install our headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install our node and library
install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
