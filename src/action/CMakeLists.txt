cmake_minimum_required(VERSION 3.5)
project(action)

set(CMAKE_BUILD_TYPE Debug)

# 1) Trouver Ament + dépendances
find_package(ament_cmake            REQUIRED)
find_package(rclcpp                 REQUIRED)
find_package(std_msgs               REQUIRED)
find_package(sensor_msgs            REQUIRED)
find_package(geometry_msgs          REQUIRED)
find_package(visualization_msgs     REQUIRED)
find_package(nav_msgs               REQUIRED)
find_package(std_srvs               REQUIRED)
find_package(tf2                    REQUIRED)
find_package(tf2_geometry_msgs      REQUIRED)
#find_package(action_complete        REQUIRED)

# 2) Inclure vos headers publics
include_directories(
  include
)

# 3) Executables
add_executable(rotation_node                    src/rotation_node.cpp)
add_executable(action_node                      src/action_node.cpp)
add_executable(translation_node                 src/translation_node.cpp)
add_executable(robot_moving_node                src/robot_moving_node.cpp)
add_executable(robot_moving_always_false_node   src/robot_moving_always_false_node.cpp)

# 4) Lier les exécutables à la libdatmo.a
target_link_libraries(rotation_node ${CMAKE_SOURCE_DIR}/lib/libaction.a)
target_link_libraries(action_node  ${CMAKE_SOURCE_DIR}/lib/libaction.a)
target_link_libraries(translation_node  ${CMAKE_SOURCE_DIR}/lib/libaction.a)
target_link_libraries(robot_moving_node  ${CMAKE_SOURCE_DIR}/lib/libaction.a)

ament_target_dependencies(rotation_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs std_srvs tf2 tf2_geometry_msgs)
ament_target_dependencies(action_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs std_srvs tf2 tf2_geometry_msgs)
ament_target_dependencies(translation_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs std_srvs tf2 tf2_geometry_msgs)
ament_target_dependencies(robot_moving_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs std_srvs tf2 tf2_geometry_msgs)
ament_target_dependencies(robot_moving_always_false_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs std_srvs tf2 tf2_geometry_msgs)

# 8) Installer les exécutables
install(
  TARGETS rotation_node action_node translation_node robot_moving_node robot_moving_always_false_node
    DESTINATION lib/${PROJECT_NAME}
)

# 9) Exporter les dépendances
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_srvs
  tf2
)

# 10) Finaliser le package
ament_package()
