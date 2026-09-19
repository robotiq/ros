find_package(ament_cmake_gtest REQUIRED)

# Compat shims over the ros2_control command-handle API. Header-only, so the
# test needs the include path but nothing to link against.
#
# Humble EOL: delete this block with the shims and their test.
ament_add_gtest(test_ros2_control_compat test_ros2_control_compat.cpp)
target_include_directories(test_ros2_control_compat
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

find_package(rclcpp REQUIRED)

ament_add_gtest(test_gripper_status test_gripper_status.cpp)
target_include_directories(test_gripper_status
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_gripper_status
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ${robotiq_msgs_TARGETS}
)

find_package(hardware_interface REQUIRED)

ament_add_gtest(test_gripper_status_broadcaster test_gripper_status_broadcaster.cpp)
target_include_directories(test_gripper_status_broadcaster
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_gripper_status_broadcaster
  ${PROJECT_NAME}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  ${robotiq_msgs_TARGETS}
)
