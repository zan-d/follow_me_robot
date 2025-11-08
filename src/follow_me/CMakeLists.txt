cmake_minimum_required(VERSION 3.5)
project(follow_me)

set(CMAKE_BUILD_TYPE Debug)

# 1) Trouver Ament, rclcpp et le package follow_me_complete
find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(visualization_msgs   REQUIRED)
find_package(nav_msgs             REQUIRED)
#find_package(follow_me_complete REQUIRED)  # apporte datmo et toutes ses dépendances

# 2) Inclure vos headers (si besoin)
include_directories(
  include
)

# 3) Executables
add_executable(detection_node src/detection_node.cpp src/datmo.cpp)
add_executable(tracking_node src/tracking_node.cpp src/datmo.cpp)
add_executable(obstacle_detection_node src/obstacle_detection_node.cpp src/datmo.cpp)

# 4) Lier la librairie datmo exportée par follow_me_complete
target_link_libraries(detection_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdatmo.a)
target_link_libraries(tracking_node  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdatmo.a)
target_link_libraries(obstacle_detection_node  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdatmo.a)

# 5) Dépendances Ament
ament_target_dependencies(detection_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs)
ament_target_dependencies(tracking_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs)
ament_target_dependencies(obstacle_detection_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs)

# 6) Installation
install(
  TARGETS
    detection_node
    tracking_node
    obstacle_detection_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 7) Package
ament_package()
