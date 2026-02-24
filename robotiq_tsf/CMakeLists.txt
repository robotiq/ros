cmake_minimum_required(VERSION 3.16)
project(robotiq_tsf LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

set(MSG_FILES
  "msg/Accelerometer.msg"
  "msg/Data1.msg"
  "msg/Data3.msg"
  "msg/Data3f.msg"
  "msg/Data4f.msg"
  "msg/Dynamic.msg"
  "msg/EulerAngle.msg"
  "msg/Gyroscope.msg"
  "msg/Magnetometer.msg"
  "msg/Quaternion.msg"
  "msg/Sensor.msg"
  "msg/StaticData.msg"
  "msg/Taxels.msg"
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
  ${LIBUSB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_algorithms
  src/MadgwickAHRS.cpp
  src/MadgwickAHRS2.cpp
)

target_include_directories(${PROJECT_NAME}_algorithms
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_algorithms
  PUBLIC
    ${OpenCV_LIBRARIES}
)

add_executable(poll_data_node src/PollData.cpp)
target_include_directories(poll_data_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(poll_data_node
  ${PROJECT_NAME}_algorithms
  ${cpp_typesupport_target}
  ${LIBUSB_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Threads::Threads
)

ament_target_dependencies(poll_data_node
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

add_executable(tactile_total_sum_node src/tactile_total_sum.cpp)
target_include_directories(tactile_total_sum_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(tactile_total_sum_node
  ${PROJECT_NAME}_algorithms
  ${cpp_typesupport_target}
  ${OpenCV_LIBRARIES}
  Threads::Threads
)

ament_target_dependencies(tactile_total_sum_node
  rclcpp
  std_msgs
  ament_index_cpp
)

install(TARGETS
  ${PROJECT_NAME}_algorithms
  poll_data_node
  tactile_total_sum_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_export_dependencies(
  OpenCV
  ament_index_cpp
  rclcpp
  rosidl_default_runtime
  std_msgs
  sensor_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_algorithms)

ament_package()
