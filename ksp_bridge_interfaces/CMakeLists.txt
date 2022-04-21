cmake_minimum_required(VERSION 3.5)
project(ksp_bridge_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CelestialBodies.msg
  msg/CelestialBody.msg
  msg/CmdRotation.msg
  msg/CmdThrottle.msg
  msg/Control.msg
  msg/Flight.msg
  msg/Orbit.msg
  msg/Part.msg
  msg/Parts.msg
  msg/Resource.msg
  msg/Vessel.msg
  srv/Activation.srv
  srv/String.srv
  srv/Switch.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
