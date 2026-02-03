cmake_minimum_required(VERSION 3.8)
project(barracuda_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(thruster_manager REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)

# Try to find control-toolbox but make it optional
find_package(ct_core QUIET)
find_package(ct_optcon QUIET)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetThrustZero.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
)

# Declare executables
add_executable(lqr_node src/lqr_node.cpp src/auv_lqr.cpp)
add_executable(pid_node src/pid_node.cpp)

# Dependencies
ament_target_dependencies(lqr_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  thruster_manager
)

ament_target_dependencies(pid_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link libraries
target_link_libraries(lqr_node
  blasfeo
  hpipm
  Eigen3::Eigen
)

target_link_libraries(pid_node
  Eigen3::Eigen
)

# Make sure the service interface is built before the executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lqr_node "${cpp_typesupport_target}")
target_link_libraries(pid_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  lqr_node
  pid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/test_target_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
