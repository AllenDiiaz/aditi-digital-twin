cmake_minimum_required(VERSION 3.8)
project(gantry_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Motion backend: source files implementing gantry_bridge::create_motion_backend()
# (see include/gantry_bridge/motion_backend.hpp) and the libraries they need.
# Both are semicolon-separated lists.
set(GANTRY_BACKEND_SOURCES "" CACHE STRING "Source files implementing gantry_bridge::create_motion_backend()")
set(GANTRY_BACKEND_LIBS "" CACHE STRING "Libraries required by the backend")

if(NOT GANTRY_BACKEND_SOURCES)
  message(FATAL_ERROR
    "A motion backend implementing gantry_bridge::create_motion_backend() is required.\n"
    "Pass its source files (and any libraries it needs) at configure time, e.g.:\n"
    "  colcon build --cmake-args -DGANTRY_BACKEND_SOURCES=/path/to/backend.cpp\n"
    "  colcon build --cmake-args -DGANTRY_BACKEND_SOURCES=/path/to/backend.cpp -DGANTRY_BACKEND_LIBS=/path/to/libcontroller.so")
endif()

add_executable(gantry_bridge_node
  src/gantry_bridge_node.cpp
  ${GANTRY_BACKEND_SOURCES}
)

target_include_directories(gantry_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(gantry_bridge_node
  ${GANTRY_BACKEND_LIBS}
)

ament_target_dependencies(gantry_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

install(TARGETS
  gantry_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
