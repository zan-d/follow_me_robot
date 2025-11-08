cmake_minimum_required(VERSION 3.5)
project(tutorial_ros)

set(CMAKE_BUILD_TYPE Debug)

# 1) Trouver Ament, rclcpp et le package follow_me_complete
find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(visualization_msgs   REQUIRED)
find_package(nav_msgs             REQUIRED)


# 2) Inclure vos headers (si besoin)
include_directories(
)

# 3) Executables
add_executable(laser_graphical_display_node src/laser_graphical_display_node.cpp)

# 4) Lier la librairie datmo exportée par follow_me_complete


# 5) Dépendances Ament
ament_target_dependencies(laser_graphical_display_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs)


# 6) Installation
install(
  TARGETS
    laser_graphical_display_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 7) Package
ament_package()
