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
find_package(tf2_eigen REQUIRED)

# find local dependencies
find_package(px4_ros_com REQUIRED)
find_package(px4_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# libraries
add_library(controller_lib src/controller_base.cpp src/simple_controller.cpp)
target_include_directories(controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(controller_lib rclcpp px4_ros_com px4_msgs Eigen3 geometry_msgs tf2_eigen)

# executables
add_executable(takeoff_example src/takeoff_example.cpp)
target_link_libraries(takeoff_example controller_lib)

add_executable(test_command src/test_command.cpp)
target_link_libraries(test_command controller_lib)

add_executable(rc_controller src/rc_controller.cpp)
target_link_libraries(rc_controller controller_lib)

# installation
install(TARGETS
    controller_lib
    takeoff_example
    test_command
    rc_controller
    DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
