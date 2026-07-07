find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(test_madgwick_ahrs
  test_madgwick_ahrs.cpp
  gtest_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
target_link_libraries(test_madgwick_ahrs ${PROJECT_NAME}_algorithms)

# Unit tests for scripts/tactile_viz_node (drives the node callbacks directly;
# needs this package's generated Python messages, hence the deferred import
# path added by colcon at test time).
ament_add_pytest_test(test_tactile_viz test_tactile_viz.py TIMEOUT 120)

ament_add_gtest(test_serial_port
  test_serial_port.cpp
  gtest_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
target_link_libraries(test_serial_port ${PROJECT_NAME}_serial)
