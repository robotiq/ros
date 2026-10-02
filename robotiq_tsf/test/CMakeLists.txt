find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

# One gtest binary for the whole package, so the suite runs — and debugs — as a
# single target instead of one per file. Pick a subset with --gtest_filter.
#
# poll_data_sdk_lib already links ${PROJECT_NAME}_algorithms, so linking it
# covers the orientation filter too; header include paths come with it.
ament_add_gtest(${PROJECT_NAME}_tests
  test_quat_helpers.cpp         # ZYX Euler extraction, quaternion conventions
  test_orientation_filter.cpp   # conventions + correction envelope, every filter
  test_madgwick_ahrs.cpp        # MadgwickFilter numeric regression
  test_filter_parity.cpp        # OrientationFilter vs MadgwickFilter, same input
  test_device_autodetect.cpp    # udev symlink / sysfs-tty descriptor scanning
  test_sdk_bridge.cpp           # SDK Fingers -> ROS message mapping
  test_fusion.cpp               # dt derivation + clamp, still-detection, bias trim
  test_poll_data_sdk_node.cpp   # pins the node's advertised topic/service set
)
target_link_libraries(${PROJECT_NAME}_tests poll_data_sdk_lib)

# Unit tests for scripts/tactile_viz_node (drives the node callbacks directly;
# needs this package's generated Python messages, hence the deferred import
# path added by colcon at test time).
ament_add_pytest_test(test_tactile_viz test_tactile_viz.py TIMEOUT 120)
