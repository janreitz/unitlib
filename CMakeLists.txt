cmake_minimum_required(VERSION 3.14)
project(unitlib VERSION 0.0.1 DESCRIPTION "Unit Library")

# Create an interface library target for your header-only library
add_library(unitlib INTERFACE)

target_compile_features(unitlib INTERFACE cxx_std_20)

add_subdirectory(tests)

# Specify the include directory for the interface library
target_include_directories(unitlib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>  # <prefix>/include when installed
)


enable_testing()

# Installation rules for the header file
install(FILES include/unitlib.hpp DESTINATION include)

# Installation rules for the library target (it will not actually install any files for an interface library, but it will export the target)
install(TARGETS unitlib EXPORT unitlibTargets)

# Export the targets to a script
install(EXPORT unitlibTargets
    FILE unitlibTargets.cmake
    DESTINATION lib/cmake/unitlib
)

# Include CMake package configuration helpers
include(CMakePackageConfigHelpers)

# Create the unitlibConfig.cmake file with the right paths
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/unitlibConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/unitlibConfig.cmake"
    INSTALL_DESTINATION lib/cmake/unitlib
)

# Create a unitlibConfigVersion.cmake file
write_basic_package_version_file(
    "unitlibConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Create a unitlibConfig.cmake file for the install tree
install(FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/unitlibConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/unitlibConfigVersion.cmake"
    DESTINATION lib/cmake/unitlib
)
