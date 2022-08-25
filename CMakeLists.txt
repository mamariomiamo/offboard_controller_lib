cmake_minimum_required(VERSION 3.10)
project(offboard_controller)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

# helper lib
add_library(helper src/helper.cpp)

target_include_directories(helper PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(helper Eigen3::Eigen)

# offb_ctrl lib
add_library(offb_ctrl src/offb_ctrl.cpp)
target_link_libraries(offb_ctrl helper)

# main test node
add_executable(main src/test.cpp)
target_link_libraries(main offb_ctrl)