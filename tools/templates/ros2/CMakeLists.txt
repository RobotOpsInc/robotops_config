cmake_minimum_required(VERSION 3.16)
project(robotops-config VERSION {{VERSION}})

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

# Source files
set(ROBOTOPS_CONFIG_SOURCES
  generated/sdks/cpp/proto/robotops/config/v1/config.pb.cc
)

# Create library target
add_library(${PROJECT_NAME} SHARED ${ROBOTOPS_CONFIG_SOURCES})

# Set C++ standard
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# Include directories for build
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated/sdks/cpp>
    $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

# Link protobuf libraries
# Use variable-based linking for compatibility with older FindProtobuf modules
# that don't create the Protobuf::libprotobuf imported target
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${Protobuf_LIBRARIES}
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY generated/sdks/cpp/proto/robotops
  DESTINATION include
)

# Export CMake targets so downstream packages can use find_package(robotops-config)
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Create and install CMake config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${CMAKE_PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(Protobuf)
ament_export_include_directories(include)

# Package
ament_package()
