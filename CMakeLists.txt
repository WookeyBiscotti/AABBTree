cmake_minimum_required(VERSION 3.0)
project(aabb_tree)

set(CMAKE_CXX_STANDARD 20)


add_library(aabb_tree INTERFACE)
target_include_directories(aabb_tree INTERFACE include)


set(BUILD_TESTS OFF CACHE BOOL "Build tests")
if(${BUILD_TESTS})
	add_subdirectory(tests)
endif()

set(BUILD_EXAMPLE OFF CACHE BOOL "Build example")
if(${BUILD_EXAMPLE})
	add_subdirectory(example)
endif()