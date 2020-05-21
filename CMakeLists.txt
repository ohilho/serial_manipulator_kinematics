cmake_minimum_required(VERSION 3.1.0)

# your project name
set(project_name hope_you_love_coding)

# build option
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

# Export your compile_commands.json file to build directory
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++ standard
set(CMAKE_CXX_STANDARD 11)

# declare yor project
project(${project_name})

# global include path setting
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)

# output directory redirection
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

# add libexample 
# add_library(model_descriptor SHARED src/smk/model_descriptor/recursive_observer.cpp)

# add gtest
# add_executable(model_descriptor_test
#     test/smk/model_descriptor/recursive_observer_test.cpp
# )
# target_link_libraries(model_descriptor_test 
#     model_descriptor 
#     ${GTEST_LIBRARY}
#     ${GTEST_MAIN_LIBRARY}
#     pthread
# )
# add_dependencies(model_descriptor_test model_descriptor)
