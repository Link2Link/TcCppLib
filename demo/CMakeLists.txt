cmake_minimum_required(VERSION 3.16)

project(project_demo)
message(STATUS "---------------------------")
message(STATUS "Current : ${PROJECT_NAME}")

add_executable(devlop ${SRC} main.cpp ppxlog.cpp)
target_include_directories(devlop PUBLIC ${PROJECT_TOP_DIR}/demo)