cmake_minimum_required(VERSION 3.8)
project(isaacsim_turtlebot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)


set(ROS2_PACKAGES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
  sensor_msgs)

include_directories(${PROJECT_SOURCE_DIR}/include)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(isaac_tb3_controller
        src/isaac_tb3_controller.cpp
        src/isaac_tb3_controller_main.cpp
        src/dynamic_window_approach.cpp
        src/global_path.cpp)
ament_target_dependencies(isaac_tb3_controller ${ROS2_PACKAGES})
install(TARGETS isaac_tb3_controller
  DESTINATION lib/${PROJECT_NAME})

#install launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
