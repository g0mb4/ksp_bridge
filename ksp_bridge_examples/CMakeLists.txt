cmake_minimum_required(VERSION 3.5)
project(ksp_bridge_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ksp_bridge_interfaces REQUIRED)

add_executable(kspb_up_and_down src/up_and_down.cpp)
ament_target_dependencies(kspb_up_and_down
  rclcpp
  ksp_bridge_interfaces
)

add_executable(kspb_resource_monitor src/resource_monitor.cpp)
ament_target_dependencies(kspb_resource_monitor
  rclcpp
  ksp_bridge_interfaces
)

install(TARGETS
  kspb_up_and_down
  kspb_resource_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
