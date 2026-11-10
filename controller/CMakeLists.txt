# Verify HOME_POSE and TARGET_POSE_A/B in test_jic_trajectory.py are safe for your actual robot configuration
# Set stiffness to safe values (50 Nm/rad JIC, 100 N/m CIC) in lbr_hardware_controllers.yaml
# Have an operator at the emergency stop at all times
# Test with lbr_JIC_hardware.launch.py first — simpler, joint-space, no FK needed

cmake_minimum_required(VERSION 3.8)
project(lbr_impedance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(lbr_ros2_control REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CartesianTrajectoryExecution.action"
  "action/JointTrajectoryExecution.action"
  "msg/NullspaceDiagnostics.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# Controller library
add_library(${PROJECT_NAME}_controllers SHARED
  src/joint_impedance_controller.cpp
  src/cartesian_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME}_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_controllers
  rclcpp
  rclcpp_action
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  kdl_parser
  orocos_kdl
  Eigen3
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_kdl
  lbr_ros2_control
  urdf
  visualization_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_controllers "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(controller_interface lbr_impedance_control.xml)

install(TARGETS ${PROJECT_NAME}_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE.md DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
