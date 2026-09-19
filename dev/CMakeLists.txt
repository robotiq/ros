# Developer-only aggregate: opens every C++ package in the workspace as a
# single CMake project, for IDEs that take one CMakeLists as their entry point.
#
# Not a build product. colcon never sees this directory — the COLCON_IGNORE
# beside this file stops it descending — and the real build is still colcon
# building each ament package on its own. Nothing here affects CI or Docker.
#
#   cmake --preset ide && cmake --build --preset ide-install && ctest --preset ide
#
# or point the IDE at dev/CMakeLists.txt and pick the "ide" preset (Qt Creator:
# Open File or Project). See CMakePresets.json beside this file.

cmake_minimum_required(VERSION 3.16)
project(robotiq_ros_ide LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

# Each package keeps its own project() and ament_package(); they are pulled in
# side by side purely so the code model sees them together. A package must come
# after the ones it depends on — see the alias below.
set(ROBOTIQ_IDE_PACKAGES
  ../robotiq_tsf
  ../grippers/robotiq_driver          # brings extern/grippers (the SDK) with it
  ../grippers/robotiq_msgs
  ../grippers/robotiq_controllers
  ../grippers/robotiq_hardware_tests
)

foreach(package IN LISTS ROBOTIQ_IDE_PACKAGES)
  get_filename_component(name ${package} NAME)
  add_subdirectory(${package} ${CMAKE_CURRENT_BINARY_DIR}/${name})

  # Packages reach each other through the namespaced target their config file
  # exports. There is no config file in-tree, so publish the same name as an
  # alias for the package that follows to link against.
  if(TARGET ${name} AND NOT TARGET ${name}::${name})
    # A package name can also belong to an executable or to a utility target
    # generated for its messages; only a library can be aliased.
    get_target_property(target_type ${name} TYPE)
    if(target_type MATCHES "LIBRARY$")
      add_library(${name}::${name} ALIAS ${name})
    endif()
  endif()
endforeach()
