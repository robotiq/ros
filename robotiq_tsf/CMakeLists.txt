cmake_minimum_required(VERSION 3.16)
project(robotiq_tsf LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

# Tactile sensors SDK (extern/tactile_sensors submodule) — provides the packet
# parsing that the SDK-backed poller (poll_data_sdk_node) relies on. Built as a
# small static lib from the single SDK translation unit.
set(TSF_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../extern/tactile_sensors/sdk_cpp")
if(NOT EXISTS "${TSF_SDK_DIR}/RobotiqTactileSensor.cpp")
  message(FATAL_ERROR
    "Tactile sensors SDK not found at ${TSF_SDK_DIR}. "
    "Initialize the submodule ('git submodule update --init --recursive') or, "
    "for the Docker build, ensure docker/Dockerfile COPYs it into the workspace.")
endif()
add_library(robotiq_tactile_sdk STATIC ${TSF_SDK_DIR}/RobotiqTactileSensor.cpp)
target_include_directories(robotiq_tactile_sdk
  PUBLIC ${TSF_SDK_DIR} ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(robotiq_tactile_sdk
  PUBLIC ${LIBSERIALPORT_LIBRARIES} Threads::Threads)

set(MSG_FILES
  "msg/Accelerometer.msg"
  "msg/Data1.msg"
  "msg/Data3.msg"
  "msg/Data3f.msg"
  "msg/Data4f.msg"
  "msg/Dynamic.msg"
  "msg/EulerAngle.msg"
  "msg/Gyroscope.msg"
  "msg/Quaternion.msg"
  "msg/Sensor.msg"
  "msg/StaticData.msg"
  "msg/Taxels.msg"
  "msg/Timestamp.msg"
)

set(SRV_FILES
  "srv/TactileSensors.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES
    std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
list(APPEND interface_targets_installed ${PROJECT_NAME}__rosidl_generator_c)
list(APPEND interface_targets_installed ${PROJECT_NAME}__rosidl_typesupport_introspection_c)
list(APPEND interface_targets_installed ${PROJECT_NAME}__rosidl_generator_py)

include_directories(
  include
  include/robotiq_tsf
)

# Fusion (third_party/fusion, MIT) is compiled in rather than built as its own
# library, so the exported robotiq_tsf_algorithms stays self-contained.
add_library(${PROJECT_NAME}_algorithms
  src/MadgwickAHRS.cpp
  src/euler_angles.cpp
  src/orientation_filter.cpp
  third_party/fusion/FusionAhrs.c
)

target_include_directories(${PROJECT_NAME}_algorithms
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    third_party/fusion
)

# Exact 1/sqrt instead of Fusion's default bit-trick approximation.
target_compile_definitions(${PROJECT_NAME}_algorithms PRIVATE FUSION_USE_NORMAL_SQRT)

target_link_libraries(${PROJECT_NAME}_algorithms
  PUBLIC
    Eigen3::Eigen
)

# SDK poller node logic as a static library, so the executable and the unit
# tests share a single build of the sources instead of recompiling them.
add_library(poll_data_sdk_lib STATIC
  src/poll_data_sdk.cpp
  src/device_autodetect.cpp
  src/sdk_bridge.cpp
)
target_include_directories(poll_data_sdk_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${TSF_SDK_DIR}
)
# Imported targets rather than ament_target_dependencies(): the latter is gone
# from ament_cmake in Lyrical. rclcpp::rclcpp, ament_index_cpp::ament_index_cpp
# and the ${<pkg>_TARGETS} variables exist on every distro we support.
target_link_libraries(poll_data_sdk_lib
  PUBLIC
    ${PROJECT_NAME}_algorithms
    ${cpp_typesupport_target}
    robotiq_tactile_sdk
    Threads::Threads
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

add_executable(poll_data_sdk_node src/poll_data_sdk_main.cpp)
target_link_libraries(poll_data_sdk_node poll_data_sdk_lib)

add_executable(tactile_total_sum_node src/tactile_total_sum.cpp)
target_include_directories(tactile_total_sum_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(tactile_total_sum_node
  PRIVATE
    ${PROJECT_NAME}_algorithms
    ${cpp_typesupport_target}
    Threads::Threads
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    ${std_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_algorithms
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  poll_data_sdk_node
  tactile_total_sum_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python RViz visualization node + its launch and config.
install(PROGRAMS
  scripts/tactile_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_export_dependencies(
  Eigen3
  ament_index_cpp
  rclcpp
  rosidl_default_runtime
  std_msgs
  sensor_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_algorithms)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
