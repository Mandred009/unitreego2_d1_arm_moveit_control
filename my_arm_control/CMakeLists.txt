cmake_minimum_required(VERSION 3.8)
project(my_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

# Core SDK + DDS dependencies
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# -----------------------------
# Build a shared library for Unitree Arm DDS messages
# -----------------------------
add_library(unitree_arm_msgs
  src/msg/ArmString_.cpp
  src/msg/PubServoInfo_.cpp
)

target_link_libraries(unitree_arm_msgs
  unitree_sdk2
  ddsc
  ddscxx
  pthread
  rt
)

# -----------------------------
# Executables
# -----------------------------
add_executable(move_arm src/move_arm.cpp)
ament_target_dependencies(move_arm rclcpp geometry_msgs sensor_msgs)
target_link_libraries(move_arm unitree_arm_msgs)

add_executable(joint_angle_control src/joint_angle_control.cpp)
target_link_libraries(joint_angle_control unitree_arm_msgs)

add_executable(multiple_joint_angle_control src/multiple_joint_angle_control.cpp)
target_link_libraries(multiple_joint_angle_control unitree_arm_msgs)

add_executable(joint_enable_control src/joint_enable_control.cpp)
target_link_libraries(joint_enable_control unitree_arm_msgs)

add_executable(arm_zero_control src/arm_zero_control.cpp)
target_link_libraries(arm_zero_control unitree_arm_msgs)

add_executable(get_arm_joint_angle src/get_arm_joint_angle.cpp)
target_link_libraries(get_arm_joint_angle unitree_arm_msgs)

# -----------------------------
# Install targets
# -----------------------------
install(TARGETS
  move_arm
  joint_angle_control
  multiple_joint_angle_control
  joint_enable_control
  arm_zero_control
  get_arm_joint_angle
  unitree_arm_msgs
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
