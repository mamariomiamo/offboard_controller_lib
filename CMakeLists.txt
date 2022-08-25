cmake_minimum_required(VERSION 3.10)
project(offboard_controller)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# include_directories(${EIGEN3_INCLUDE_DIRS} include)

add_library(offb_ctrl src/offb_ctrl.cpp)
add_library(helper INTERFACE)

add_executable(main src/test.cpp)

target_link_libraries(helper)
target_link_libraries(offb_ctrl helper eigen)
target_link_libraries(main offb_ctrl)

target_include_directories (main PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)