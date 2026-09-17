find_package(ament_cmake_gtest REQUIRED)

# Compat shims over the ros2_control command-handle API. Header-only, so the
# test needs the include path but nothing to link against.
#
# Humble EOL: delete this block with the shims and their test.
ament_add_gtest(test_ros2_control_compat test_ros2_control_compat.cpp)
target_include_directories(test_ros2_control_compat
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
