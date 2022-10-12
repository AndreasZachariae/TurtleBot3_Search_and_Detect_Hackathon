cmake_minimum_required(VERSION 3.5)
project(behavior_tree_coordinator)

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
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros REQUIRED)
find_package(petra_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cpp_core REQUIRED)
find_package(nav2_msgs REQUIRED)
# for python code
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
  include 
  ${behaviortree_ros_INCLUDE_DIRS}
  ${cpp_core_INCLUDE_DIRS})

 
set(DEPENDENCIES 
  rclcpp
  rclcpp_action
  behaviortree_ros
  petra_interfaces
  tf2_ros
  nav2_msgs
  cpp_core
)

file (GLOB SOURCES_ACTIONS src/actions/*.cpp)
file (GLOB SOURCES_CONDITIONS src/conditions/*.cpp)
file (GLOB SOURCES_CONTROLS src/controls/*.cpp)

add_executable(Coordinator
  src/node.cpp
  # src/examples/minimal_bt_action.cpp
  ${SOURCES_ACTIONS}
  ${SOURCES_CONDITIONS}
  ${SOURCES_CONTROLS})

target_link_libraries(Coordinator
  behaviortree_ros::behaviortree_ros_lib
  cpp_core::cpp_core_lib
)

ament_target_dependencies(Coordinator
  ${DEPENDENCIES})

add_executable(MinimalNavAction
  src/examples/minimal_nav_action_node.cpp
)

ament_target_dependencies(MinimalNavAction
  rclcpp
  rclcpp_action
  nav2_msgs
)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  behaviors
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  Coordinator
  MinimalNavAction
  DESTINATION lib/${PROJECT_NAME})

# for python code
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    ${PROJECT_NAME}/detect_ball_node.py
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()