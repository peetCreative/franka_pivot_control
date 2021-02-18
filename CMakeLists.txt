cmake_minimum_required( VERSION 3.16 )
project( franka_pivot_control )

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)

find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
message("Found Eigen Version: ${Eigen3_VERSION}")
message("Include: ${Eigen3_INCLUDE_DIRS}")
find_package( frankx REQUIRED )
find_package(Franka 0.8 REQUIRED)
message("Franka: ${Franka_FOUND}")
message("Include: ${Franka_VERSION}")
include_directories(
        "${frankx_INCLUDE_DIRS}"
        "${Franka_INCLUDE_DIRS}")

SET(franka_pivot_control_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/franka_pivot_control)
include_directories(${franka_pivot_control_INCLUDE_DIR})

SET(franka_pivot_control_LIB ${PROJECT_NAME}_lib)

add_library(
    ${franka_pivot_control_LIB} SHARED
        src/FrankaPivotControl.cpp
        src/PivotControl.cpp)

add_executable(keyboardTest
        example/keyboardTest.cpp)

target_link_libraries(${franka_pivot_control_LIB}
    Eigen3::Eigen
    ${frankx_LIBRARIES})

target_link_libraries(keyboardTest
    ${franka_pivot_control_LIB})

install( TARGETS ${franka_pivot_control_LIB}
        DESTINATION bin
        LIBRARY DESTINATION lib)

install( DIRECTORY ${franka_pivot_control_INCLUDE_DIR}
        DESTINATION include)

