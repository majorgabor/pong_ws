cmake_minimum_required(VERSION 3.8)
project(pong_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ball.msg"
  "msg/GameStatus.msg"
  "msg/Player.msg"
  "msg/PlayerAction.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
