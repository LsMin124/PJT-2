cmake_minimum_required(VERSION 3.8)
project(amr_agent_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MovePrimitive.action"
)

ament_package()
