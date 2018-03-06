cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

# Name of project
project(mavss)

# Project dependencies
find_package(Threads REQUIRED)

# CMake Modules
#include(TestBigEndian)

# Use glob to get the list of all source files.
file(GLOB_RECURSE APP_SOURCES "src/*.cpp")

# Use glob to get the list of all header files.
file(GLOB_RECURSE APP_HEADERS "include/*.hpp")
file(GLOB_RECURSE LIB_HEADERS "lib/mavlink/*.hpp")

# Project options
option(BUILD_TESTS				"Build tests."				OFF)
#option(BUILD_DOCS				"Build documentation."		OFF)

# Project properties
set(CMAKE_CXX_STANDARD			14)
set(CMAKE_CXX_EXTENSIONS		OFF)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Executable target
add_executable(${PROJECT_NAME} ${APP_SOURCES} ${APP_HEADERS} ${LIB_HEADERS})

target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)

# Library additional folders to include
target_include_directories(${PROJECT_NAME} PRIVATE "include/")
target_include_directories(${PROJECT_NAME} PRIVATE "lib/mavlink/common/")

# Check build settings
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE DEBUG)
endif()

# OS & compiler specific options
if(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "/Ox")
	set(CMAKE_CXX_FLAGS_DEBUG "/Zi")
elseif(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -ffast-math")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3")
	set(CMAKE_CXX_FLAGS_DEBUG "-g3")
elseif(APPLE)
	set(CMAKE_CXX_FLAGS )
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