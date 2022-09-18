# Copyright 2022 Stanford ASL
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(asl_flight2)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find global dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# find local dependencies
find_package(px4_msgs REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# libraries
add_library(control src/control/controller_base.cpp)
target_include_directories(control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(control
    rclcpp px4_msgs Eigen3 geometry_msgs tf2_eigen)
ament_export_targets(controlTarget HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs px4_msgs)

add_library(mocap src/mocap/mocap_relay.cpp)
ament_target_dependencies(mocap
    rclcpp px4_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# examples
add_executable(example_takeoff src/examples/takeoff.cpp)
target_link_libraries(example_takeoff control)

# PS4 RC Control
add_executable(controller_ps4 src/control/controller_ps4.cpp)
ament_target_dependencies(controller_ps4 sensor_msgs)
target_link_libraries(controller_ps4 control)

# Simulation grountruth publisher
find_package(gazebo QUIET)
if(gazebo_FOUND)
    add_executable(mocap_relay_sim src/mocap/mocap_relay_sim.cpp)
    target_link_libraries(mocap_relay_sim ${GAZEBO_LIBRARIES} mocap)
    target_include_directories(mocap_relay_sim PUBLIC ${GAZEBO_INCLUDE_DIRS})
    install(TARGETS mocap_relay_sim DESTINATION lib/${PROJECT_NAME})
endif()

# VRPN mocap relay
add_executable(mocap_relay_vrpn src/mocap/mocap_relay_vrpn.cpp)
target_link_libraries(mocap_relay_vrpn mocap)

# install executable
install(TARGETS
    example_takeoff
    controller_ps4
    mocap_relay_vrpn
    DESTINATION lib/${PROJECT_NAME})

# install header
install(
    DIRECTORY include/
    DESTINATION include)

# install lib
install(
    TARGETS control
    EXPORT controlTarget
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# install misc
install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME})

# export dependencies for Eigen3
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
