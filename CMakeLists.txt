cmake_minimum_required( VERSION 3.16 )
project( franka_pivot_control )

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
message("Found Eigen Version: ${Eigen3_VERSION}")
message("Include: ${Eigen3_INCLUDE_DIRS}")
find_package( frankx REQUIRED )
find_package(Franka 0.8 REQUIRED)
message("Franka: ${Franka_FOUND}")
message("Include: ${Franka_VERSION}")

add_subdirectory(pivot_control_messages)

include_directories(
        "${frankx_INCLUDE_DIRS}"
        "${Franka_INCLUDE_DIRS}"
        "pivot_control_messages/include")

SET(franka_pivot_control_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/franka_pivot_control)
include_directories(${franka_pivot_control_INCLUDE_DIR})

SET(franka_pivot_control_LIB ${PROJECT_NAME})

add_library(
    ${franka_pivot_control_LIB} SHARED
        src/FrankaPivotController.cpp)

add_executable(keyboardTest
        example/keyboardTest.cpp)

target_link_libraries(${franka_pivot_control_LIB}
    Eigen3::Eigen
    ${frankx_LIBRARIES}
        ${Franka_LIBRARIES}
        "pivot_control_messages")

target_link_libraries(keyboardTest
    ${franka_pivot_control_LIB})

install( TARGETS ${franka_pivot_control_LIB}
        DESTINATION lib)

install( FILES ${franka_pivot_control_INCLUDE_DIR}/FrankaPivotController.h
        DESTINATION include)

