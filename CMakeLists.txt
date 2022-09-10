cmake_minimum_required(VERSION 3.8)
project(asl_flight2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find global dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# find local dependencies
find_package(px4_ros_com REQUIRED)
find_package(px4_msgs REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# libraries
add_library(autonomy_lib src/control/controller_base.cpp src/mocap/mocap_relay.cpp)
target_include_directories(autonomy_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(autonomy_lib
    rclcpp px4_ros_com px4_msgs Eigen3 geometry_msgs tf2_eigen tf2 tf2_ros tf2_geometry_msgs)

# examples
add_executable(example_takeoff src/examples/takeoff.cpp)
target_link_libraries(example_takeoff autonomy_lib)

# PS4 RC Control
add_executable(controller_ps4 src/control/controller_ps4.cpp)
target_link_libraries(controller_ps4 autonomy_lib)

# Simulation grountruth publisher
find_package(gazebo)
if(gazebo_FOUND)
    add_executable(mocap_relay_sim src/mocap/mocap_relay_sim.cpp)
    target_link_libraries(mocap_relay_sim ${GAZEBO_LIBRARIES} autonomy_lib)
    target_include_directories(mocap_relay_sim PUBLIC ${GAZEBO_INCLUDE_DIRS})
    install(TARGETS mocap_relay_sim DESTINATION lib/${PROJECT_NAME})
endif()

# installation
install(TARGETS
    autonomy_lib
    controller_ps4
    example_takeoff
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME})

# export dependencies for Eigen3
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
