cmake_minimum_required(VERSION 3.17)
project(indexer)

set(CMAKE_CXX_STANDARD 20)


add_library(indexer INTERFACE)
target_include_directories(indexer INTERFACE include)


set(BUILD_TESTS OFF CACHE BOOL "Build tests")

if(${BUILD_TESTS})
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

	add_executable(indexer_TESTS tests/catch2_main.cpp tests/tests.cpp)
	add_dependencies(indexer_TESTS catch2)

	target_link_libraries(indexer_TESTS indexer)
endif()