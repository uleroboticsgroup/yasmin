cmake_minimum_required(VERSION 3.5)
project(yasmin_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/Transition.msg"
  "msg/StateMachine.msg"
)

ament_package()
