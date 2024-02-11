test: include/unitlib.hpp tests/test.cpp
	g++ -std=c++20 -Wall -Wextra -Wpedantic -Werror tests/test.cpp