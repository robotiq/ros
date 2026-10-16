find_package(ament_cmake_gtest REQUIRED)

# Compat shims over the ros2_control handle, realtime_tools container and
# gripper action result APIs. Header-only, so the test needs the include path
# but nothing to link against.
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

ament_add_gtest(test_robotiq_activation_controller test_robotiq_activation_controller.cpp)
target_include_directories(test_robotiq_activation_controller
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_robotiq_activation_controller
  ${PROJECT_NAME}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  rclcpp::rclcpp
)

ament_add_gtest(test_object_status_goal test_object_status_goal.cpp)
target_include_directories(test_object_status_goal
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_object_status_goal
  ${PROJECT_NAME}
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  ${robotiq_msgs_TARGETS}
)

find_package(rclcpp_action REQUIRED)

# Humble EOL: delete this block.
if(gripper_controllers_FOUND)
  ament_add_gtest(test_gripper_command_controller test_gripper_command_controller.cpp)
  target_include_directories(test_gripper_command_controller
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )
  target_link_libraries(test_gripper_command_controller
    ${PROJECT_NAME}
    controller_interface::controller_interface
    gripper_controllers::gripper_action_controller
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${robotiq_msgs_TARGETS}
  )
endif()

# Humble EOL: simplify — drop the condition.
if(parallel_gripper_controller_FOUND)
  ament_add_gtest(test_gripper_action_controller test_gripper_action_controller.cpp)
  target_include_directories(test_gripper_action_controller
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )
  target_link_libraries(test_gripper_action_controller
    ${PROJECT_NAME}
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    parallel_gripper_controller::parallel_gripper_action_controller
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${robotiq_msgs_TARGETS}
  )
endif()
