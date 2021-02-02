cmake_minimum_required(VERSION 3.17)
project(aabb_tree_tests)

set(CMAKE_CXX_STANDARD 20)

include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)

ExternalProject_Add(catch2
		GIT_REPOSITORY https://github.com/catchorg/Catch2.git
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
		GIT_TAG v2.13.4
		)

ExternalProject_Get_Property(catch2 source_dir)
set(CATCH2_DIR ${source_dir} CACHE INTERNAL "Path to include folder for Catch2")
include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)

add_executable(aabb_tree_tests catch2_main.cpp tests.cpp)
add_dependencies(aabb_tree_tests catch2)

target_link_libraries(aabb_tree_tests aabb_tree)