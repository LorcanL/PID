cmake_minimum_required(VERSION 3.16.3)
project(pid)
file(GLOB_RECURSE SOURCES "${PROJECT_SOURCE_DIR}/pid/*.cpp" "${PROJECT_SOURCE_DIR}/KinematicModel/*.cpp")
set(HEAD_DIR ${PROJECT_SOURCE_DIR}/include)
message("1、PROJECT_SOURCE_DIR is: ${PROJECT_SOURCE_DIR}")
message("SOURCES is: ${SOURCES}")
message("HEAD_DIR is: ${HEAD_DIR}")
add_subdirectory(src)

