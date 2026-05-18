cmake_minimum_required(VERSION 3.16)
project(motion_spec_mj_kdl_target LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED CONFIG)

set(MJ_KDL_WRAPPER_SOURCE_DIR "" CACHE PATH "Path to the mj_kdl_wrapper source tree")
if(MJ_KDL_WRAPPER_SOURCE_DIR)
    add_subdirectory("${MJ_KDL_WRAPPER_SOURCE_DIR}" mj_kdl_wrapper_build)
    set(MJ_KDL_WRAPPER_TARGET mj_kdl_wrapper)
else()
    find_package(mj_kdl_wrapper REQUIRED CONFIG)
    set(MJ_KDL_WRAPPER_TARGET mj_kdl_wrapper::mj_kdl_wrapper)
endif()

add_executable(main
    ref_main.cpp
)
target_include_directories(main PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/headers
)
target_link_libraries(main PRIVATE
    ${MJ_KDL_WRAPPER_TARGET}
    orocos-kdl
)
target_compile_options(main PRIVATE -Wall -Wextra)

add_executable(pick_place_sim
    pick_place_sim.cpp
)
target_include_directories(pick_place_sim PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/headers
)
target_link_libraries(pick_place_sim PRIVATE
    ${MJ_KDL_WRAPPER_TARGET}
    orocos-kdl
)
target_compile_options(pick_place_sim PRIVATE -Wall -Wextra)
