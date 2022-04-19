cmake_minimum_required(VERSION 3.5)
project(ksp_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ksp_bridge_interfaces REQUIRED)

include(FindProtobuf)
find_package(Protobuf 3.2 REQUIRED)

find_library(krpc_LIBRARY NAMES krpc
  PATHS /usr/lib
        /usr/lib64
        /usr/local/lib
        /usr/local/lib64
)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/ksp_bridge_node.cpp 
  src/ksp_bridge.cpp
  src/utils.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  ksp_bridge_interfaces
)

target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARY}
  ${krpc_LIBRARY}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
