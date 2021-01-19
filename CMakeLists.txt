cmake_minimum_required(VERSION 3.17)
project(indexer)

set(CMAKE_CXX_STANDARD 20)


add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_include_directories(${CMAKE_PROJECT_NAME} INTERFACE include)


set(BUILD_TESTS OFF CACHE BOOL "Build tests")

if(${BUILD_TESTS})
	include(ExternalProject)
	ExternalProject_Add(catch2
			PREFIX ${PROJECT_SOURCE_DIR}/tests/3rdparty/Catch2
			GIT_REPOSITORY https://github.com/catchorg/Catch2.git
			GIT_TAG v2.13.4
			CONFIGURE_COMMAND ""
			BUILD_COMMAND ""
			INSTALL_COMMAND ""
			LOG_DOWNLOAD ON
			)

	ExternalProject_Get_Property(catch2 source_dir)
	set(CATCH2_DIR ${source_dir} CACHE INTERNAL "Path to include folder for Catch2")
	add_subdirectory(${CATCH2_DIR})

	add_executable(${CMAKE_PROJECT_NAME}_TESTS tests/catch2_main.cpp tests/tests.cpp)

	add_dependencies(${CMAKE_PROJECT_NAME}_TESTS catch2)

	target_link_libraries(${CMAKE_PROJECT_NAME}_TESTS Catch2::Catch2 ${CMAKE_PROJECT_NAME})
endif()