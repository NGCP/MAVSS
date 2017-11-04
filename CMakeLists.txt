cmake_minimum_required(VERSION 3.9.0)

# Name of project
project(mavss)

# Project dependencies
find_package(Threads REQUIRED)
# find_package(Boost REQUIRED)

# CMake Modules
#include(TestBigEndian)

# Use glob to get the list of all source files.
file(GLOB_RECURSE LIB_SOURCES "src/*.cpp")

# Use glob to get the list of all header files.
file(GLOB_RECURSE LIB_HEADERS "src/*.hpp")

# Project options
option(BUILD_TESTS				"Build tests."				OFF)
#option(BUILD_DOCS				"Build documentation."		OFF)

# Project properties
set(CMAKE_CXX_STANDARD			11)
set(CMAKE_CXX_EXTENSIONS		OFF)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Library additional folders to include
include_directories("include/")
include_directories("lib/mavlink/")

# Executable target
add_executable(mavss ${LIB_SOURCES} ${LIB_HEADERS})

# Check build settings
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE DEBUG)
endif()

# OS & compiler specific options
if(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall /Zi")
	set(CMAKE_CXX_FLAGS_RELEASE "/Ox")
	set(CMAKE_CXX_FLAGS_DEBUG "/Zi")
elseif(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -ffast-math")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3")
	set(CMAKE_CXX_FLAGS_DEBUG "-g3")
elseif(APPLE)
	# TODO: Find Compiler settings for Mac OS X
endif()

#if(BUILD_TESTS)
#include(GoogleTest)
#	add_subdirectory(lib/googletest)
#	enable_testing()
#	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
#
#	add_executable(GetNifVersionTest test/get_nif_version_test.cpp)
#	target_link_libraries(GetNifVersionTest gtest gtest_main)
#	target_link_libraries(GetNifVersionTest niflib)
#
#	add_test(get_nif_version GetNifVersionTest "${CMAKE_CURRENT_LIST_DIR}/data/")
#endif()