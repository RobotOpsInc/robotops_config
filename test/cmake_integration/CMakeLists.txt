cmake_minimum_required(VERSION 3.16)
project(robotops-config-integration-test)

# Test 1: Can we find the package?
find_package(robotops-config REQUIRED)

if(NOT robotops-config_FOUND)
  message(FATAL_ERROR "robotops-config not found")
endif()

message(STATUS "✓ robotops-config found: ${robotops-config_VERSION}")

# Test 2: Does the target exist?
if(NOT TARGET robotops-config::robotops-config)
  message(FATAL_ERROR "Target robotops-config::robotops-config not found")
endif()

message(STATUS "✓ Target robotops-config::robotops-config exists")

# Test 3: Can we get target properties?
get_target_property(_include_dirs robotops-config::robotops-config INTERFACE_INCLUDE_DIRECTORIES)
if(NOT _include_dirs)
  message(FATAL_ERROR "No include directories exported")
endif()

message(STATUS "✓ Include directories: ${_include_dirs}")

get_target_property(_link_libs robotops-config::robotops-config INTERFACE_LINK_LIBRARIES)
if(NOT _link_libs)
  message(FATAL_ERROR "No link libraries exported")
endif()

message(STATUS "✓ Link libraries: ${_link_libs}")

# Test 4: Can we compile a minimal program using it?
add_executable(test_robotops_config test.cpp)
target_link_libraries(test_robotops_config robotops-config::robotops-config)

message(STATUS "✓ Successfully linked test executable")
