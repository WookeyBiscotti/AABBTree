cmake_minimum_required(VERSION 3.17)

project(aabb_tree_example)

set(CMAKE_CXX_STANDARD 20)

add_executable(aabb_tree_example main.cpp)

# For stand alone example
# add_subdirectory(your_path_to_aabb_tree)

target_link_libraries(aabb_tree_example aabb_tree)
