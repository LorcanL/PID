project(pid_main)

find_package(Python3 COMPONENTS Development)
find_package(Eigen3 REQUIRED)

message("2、PROJECT_SOURCE_DIR is: ${PROJECT_SOURCE_DIR}")


include_directories(
    ${HEAD_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    /usr/include/python3.8
    )

add_executable(main main.cpp ${SOURCES})

target_link_libraries(main Python3::Python)
