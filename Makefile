test: include/unitlib.h test/test.cpp
	g++ -std=c++20 -Wall -Wextra -Wpedantic -Werror test/test.cpp