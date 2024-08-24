cmake_minimum_required(VERSION 3.5)
project(go2_gait_planner)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(go2_scratch REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoveLeg.msg"
  "msg/ParamsSet.msg"
  "msg/JointsSet.msg"
  "msg/GaitParam.msg"
)

# Ensure dependencies are found for generated messages
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${unitree_go_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${go2_scratch_INCLUDE_DIRS}
)


# Library
add_library(${PROJECT_NAME}_lib
  src/Quadruped/Link.cpp
  src/Quadruped/Joint.cpp
  src/Quadruped/Leg.cpp
  src/Quadruped/Robot.cpp
  src/Quadruped/Kinematics.cpp
  src/Low_level/StateMonitor.cpp
  src/Low_level/FakePublisher.cpp
  src/Low_level/LegMover.cpp
  src/Low_level/BodyMover.cpp
  src/Low_level/BaseGait.cpp
  src/Low_level/TrotGait.cpp
  src/motor_crc.cpp
)

# Specify dependencies
set(DEPENDENCY_LIST
  rclcpp
  std_msgs
  unitree_go
)

# Executables
add_executable(${PROJECT_NAME}_state_monitor_node src/Low_level/state_monitor_node.cpp)
add_executable(${PROJECT_NAME}_fake_publisher_node src/Low_level/fake_publisher_node.cpp)
add_executable(${PROJECT_NAME}_body_mover_node src/Low_level/body_mover_node.cpp)
add_executable(${PROJECT_NAME}_trot_gait_node src/Low_level/trot_gait_node.cpp)
# add_executable(${PROJECT_NAME}_test_node src/Low_level/test.cpp)

# Link libraries to executables
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCY_LIST})
ament_target_dependencies(${PROJECT_NAME}_state_monitor_node ${DEPENDENCY_LIST})
ament_target_dependencies(${PROJECT_NAME}_fake_publisher_node ${DEPENDENCY_LIST})
ament_target_dependencies(${PROJECT_NAME}_body_mover_node ${DEPENDENCY_LIST})
ament_target_dependencies(${PROJECT_NAME}_trot_gait_node ${DEPENDENCY_LIST})
# ament_target_dependencies(${PROJECT_NAME}_test_node ${DEPENDENCY_LIST})

target_link_libraries(${PROJECT_NAME}_state_monitor_node ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_fake_publisher_node ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_body_mover_node ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_trot_gait_node ${PROJECT_NAME}_lib)
# target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME}_lib)

# Ensure ROSIDL generated headers are included
rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_state_monitor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_fake_publisher_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_body_mover_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_trot_gait_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# rosidl_target_interfaces(${PROJECT_NAME}_test_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  ${PROJECT_NAME}_state_monitor_node
  ${PROJECT_NAME}_fake_publisher_node
  ${PROJECT_NAME}_body_mover_node
  ${PROJECT_NAME}_trot_gait_node
  # ${PROJECT_NAME}_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lib
  DESTINATION lib
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
