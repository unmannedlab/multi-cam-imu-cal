##--------------------------------------------------------------------------------------------------------------------##
##                                                                                                                    ##
##                                                    xCalibrate:                                                     ##
##                                                                                                                    ##
##                                       Kalman Filter-Based Sensor Calibration                                       ##
##                                                                                                                    ##
##                                          Copyright (C) 2022 Jacob Hartzer                                          ##
##                                                                                                                    ##
## This program is free software: you can redistribute it and/or modify it under the terms of the                     ##
## GNU General Public License as published by the Free Software Foundation, either version 3 of the License,          ##
## or (at your option) any later version.                                                                             ##
##                                                                                                                    ##
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;                          ##
## without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                          ##
## See the GNU General Public License for more details.                                                               ##
##                                                                                                                    ##
## You should have received a copy of the GNU General Public License along with this program.                         ##
## If not, see <https://www.gnu.org/licenses/>.                                                                       ##
##                                                                                                                    ##
##--------------------------------------------------------------------------------------------------------------------##

cmake_minimum_required(VERSION 3.0)
project(xcalibrate)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage" )
set(RUN_CODE_COVERAGE OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    rospy
    std_msgs
    sensor_msgs
    image_transport
    geometry_msgs
    message_generation
    cv_bridge
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
    DEPENDS
    geometry_msgs
)

###########
## Build ##
###########

# xCalibrate Build
include_directories(
    include
    src
    nodes
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

set(XCALIBRATE_SRC
    nodes/xCalibrateNode.cpp
    src/CalibrationEKF.cpp
    src/DynamicSensor.cpp
)

set(VIZ_NODE_SRC
    nodes/VizNode.cpp
)

add_executable(${PROJECT_NAME}_node
    nodes/xCalibrateNodeMain.cpp
    ${XCALIBRATE_SRC}
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN_INCLUDE_DIRS}
    Eigen3::Eigen
)

# Vizualizer Build
add_executable(viz_node 
    ${VIZ_NODE_SRC}
    nodes/VizNodeMain.cpp
)

target_link_libraries(
    viz_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS viz_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Doxygen ##
#############
SET(DOXYGEN_STUB_FILE ${PROJECT_SOURCE_DIR}/docs/empty.html)

add_custom_target(DOXYGEN
    DEPENDS ${DOXYGEN_STUB_FILE} COVERAGE
)

FIND_PACKAGE(Doxygen)
IF(NOT DOXYGEN_FOUND)
    MESSAGE(FATAL_ERROR "Doxygen is needed to build the documentation.")
ELSE()
    SET(DOXYFILE ${PROJECT_SOURCE_DIR}/.doxyfile)

    ADD_CUSTOM_COMMAND(
        OUTPUT ${DOXYGEN_STUB_FILE}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        MAIN_DEPENDENCY ${doxyfile} ${doxyfile_in}
        COMMENT "Generating HTML documentation"
    )
ENDIF()



#############
## Testing ##
#############
include(test/CodeCoverage.cmake)
append_coverage_compiler_flags()

add_subdirectory(test)
add_subdirectory(include/test)
add_subdirectory(nodes/test)
add_subdirectory(src/test)

rollup_google_tests()

# Set up coverage for test executables
setup_target_for_coverage_lcov(
    NAME COVERAGE
    EXECUTABLE ALL_GOOGLE_TESTS
    EXCLUDE "/opt/*" "/usr/*"
)

target_link_libraries(ALL_GOOGLE_TESTS 
    ${catkin_LIBRARIES}
)