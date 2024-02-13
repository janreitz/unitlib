add_executable(tests test.cpp)

# Apply compiler flags specifically to the tests target
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(MSVC)
    target_compile_options(tests PRIVATE /W4 /WX)
endif()

target_link_libraries(tests PRIVATE unitlib)

enable_testing()
add_test(NAME UnitTests COMMAND tests)
