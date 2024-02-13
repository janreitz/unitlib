# Enable CMake's testing functionality
enable_testing()

add_executable(tests test.cpp)

target_link_libraries(tests PRIVATE unitlib)

# Register the tests executable as a test so CTest can run it
add_test(NAME UnitTests COMMAND tests)
