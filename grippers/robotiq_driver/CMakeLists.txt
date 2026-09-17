# See https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.10)
# C as well as CXX: the gripper SDK added below is a C/C++ project, and CMake 4
# wants the language enabled here before the subproject asks for it.
project(robotiq_driver LANGUAGES C CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

###############################################################################
# GRIPPER SDK

# The Modbus transport, the typed register blocks and the Robotiq::Gripper
# runtime API live in the extern/grippers submodule; this package is the
# ros2_control wrapper over them.
#
# EXCLUDE_FROM_ALL skips the SDK's own install/export rules: it is compiled
# into robotiq_driver, not shipped as a second installed package.
set(GRIPPERS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../extern/grippers/sdk_cpp")
if(NOT EXISTS "${GRIPPERS_SDK_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "The gripper SDK is missing from ${GRIPPERS_SDK_DIR}. "
    "Run `git submodule update --init extern/grippers` at the repository root.")
endif()
# The SDK declares its library without STATIC or SHARED, so it follows
# BUILD_SHARED_LIBS. Nothing installs it, so a shared one would leave the
# plugin needing a library that does not ship.
set(_robotiq_saved_build_shared_libs "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(${GRIPPERS_SDK_DIR} ${CMAKE_CURRENT_BINARY_DIR}/grippers_sdk EXCLUDE_FROM_ALL)
set(BUILD_SHARED_LIBS "${_robotiq_saved_build_shared_libs}")

###############################################################################
# Robotiq driver library.

add_library(
  robotiq_driver
  SHARED
  include/robotiq_driver/gripper_scaling.hpp
  include/robotiq_driver/hardware_interface.hpp
  include/robotiq_driver/hardware_parameters.hpp
  include/robotiq_driver/rclcpp_logger.hpp
  # Humble EOL: delete the next line with the header.
  include/robotiq_driver/ros2_control_compat.hpp
  include/robotiq_driver/visibility_control.hpp
  # Signposts for the headers this package used to install, so downstream
  # code that includes one stops with an explanation instead of a
  # missing-file error. See the migration notes in the repository README.
  include/robotiq_driver/crc_utils.hpp
  include/robotiq_driver/data_utils.hpp
  include/robotiq_driver/default_driver.hpp
  include/robotiq_driver/default_driver_factory.hpp
  include/robotiq_driver/default_serial.hpp
  include/robotiq_driver/default_serial_factory.hpp
  include/robotiq_driver/driver.hpp
  include/robotiq_driver/driver_exception.hpp
  include/robotiq_driver/driver_factory.hpp
  include/robotiq_driver/fake/fake_driver.hpp
  include/robotiq_driver/serial.hpp
  include/robotiq_driver/serial_factory.hpp
  src/hardware_interface.cpp
  src/hardware_parameters.cpp
  src/rclcpp_logger.cpp
)
target_include_directories(
  robotiq_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # The SDK's headers are part of this package's public API: hardware_interface.hpp
  # holds a Robotiq::Gripper, so downstream compilations have to see it too.
  # They are installed next to ours below, hence the single install interface.
  $<BUILD_INTERFACE:${GRIPPERS_SDK_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  robotiq_driver
  PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  PRIVATE
  # The SDK's object library, not its archive: the SDK's headers ship with this
  # package, so a consumer may call any SDK entry point and every object has to
  # land in the library, not just the ones the wrapper references.
  # $<BUILD_INTERFACE:...> keeps it out of the installed export set, where there
  # would be no target for it to resolve to.
  #
  # The consequence to know about: the whole SDK symbol set is exported from a
  # dlopen'd plugin. A second package in the same process linking the SDK the
  # same way gets its own copy with independent static state, and Robotiq::Gripper
  # owns a serial link and a thread. One SDK consumer per process.
  $<BUILD_INTERFACE:Robotiq::grippers_objects>
)

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

###############################################################################
# EXPORTS

# This is necessary to allow this library’s clients to use the syntax
# target_link_libraries(client <pkg>::<target_name>)
# Without this a client cannot find this library.
# It can take an arbitrary list of targets named as EXPORT in an install call.
ament_export_targets(
  robotiq_driver_targets # Must match the EXPORT label below in the install section.
)
# Help downstream packages to find transitive dependencies i.e. export all
# dependencies required by a package to use this library.
# When a package calls find_package(epick_driver), CMake looks for a file
# called epick_driverConfig.cmake which sets up everything another project
# would need to depend on this one.
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Tell downstream packages where to find our headers.
ament_export_include_directories(
  include
)
# Tell downstream packages our libraries to link against.
ament_export_libraries(
  robotiq_driver
)

###############################################################################
# INSTALL

# Install all files of the include folder into the give destination.
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)

# The SDK headers ship alongside ours: they are reachable from this package's
# public headers, so a consumer including hardware_interface.hpp needs them.
#
# The pattern assumes the SDK is .hpp-only, which it is today. Anything else it
# starts shipping — a C shim, a generated config header — would be dropped here
# and fail at a downstream consumer's include, not in this build.
install(
    DIRECTORY ${GRIPPERS_SDK_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
    FILES_MATCHING PATTERN "*.hpp"                   # not the SDK's own CMakeLists
)

# Install our library.
install(
  TARGETS robotiq_driver
  EXPORT robotiq_driver_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

###############################################################################
# TESTS

# CTest module automatically creates a BUILD_TESTING option that selects
# whether to enable testing support (ON by default).
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
