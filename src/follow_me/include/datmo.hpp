#pragma once

#ifndef DATMO_HPP
#define DATMO_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <fstream>
#include <string>
#include <vector>
#include <cmath>

inline constexpr int tab_size = 1100;

inline constexpr float detection_threshold = 0.2f;
inline constexpr int dynamic_threshold = 75;
inline constexpr float default_cluster_threshold = 0.05f;//0.2f;

inline constexpr float leg_size_min = 0.05f;
inline constexpr float leg_size_max = 0.25f;
inline constexpr float legs_distance_min = 0.1f;
inline constexpr float legs_distance_max = 0.7f;

inline constexpr int frequency_init = 5;
inline constexpr int frequency_max = 25;

inline constexpr float uncertainty_init = 0.5f;
inline constexpr float uncertainty_min = 0.5f;
inline constexpr float uncertainty_max = 1.0f;
inline constexpr float uncertainty_inc = 0.05f;

inline constexpr float robair_size = 0.25f;
inline constexpr float goal_point_offset = 0.3f;

class Datmo : public rclcpp::Node
{
protected:
  // ROS subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_robot_moving_;

  // Visualization markers
  visualization_msgs::msg::Marker marker_field_of_view_, marker_motion_, marker_clusters_, marker_legs_, marker_persons_, marker_moving_detected_person_, marker_tracked_person_;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_field_of_view_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_motion_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_clusters_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_legs_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_persons_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_moving_detected_person_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_tracked_person_marker_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_detection_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_tracking_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_closest_obstacle_;

  // TF (disabled for now)
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Laser scan data
  int nb_beams_;
  bool init_laser_, new_laser_;
  float range_min_, range_max_;
  float angle_min_, angle_max_, angle_inc_;
  float r_[tab_size], theta_[tab_size];
  geometry_msgs::msg::Point current_scan_[tab_size];

  // Motion detection
  bool init_robot_, new_robot_;
  bool stored_background_;
  float background_[tab_size];
  bool dynamic_[tab_size];
  bool current_robot_moving_;
  bool previous_robot_moving_;

  // Clustering data
  int nb_clusters_;
  int cluster_start_[tab_size], cluster_end_[tab_size];
  float cluster_size_[tab_size];
  geometry_msgs::msg::Point cluster_middle_[tab_size];
  int cluster_dynamic_[tab_size];
  int cluster_nb_points_[tab_size];

  // Leg detection
  int nb_legs_detected_;
  geometry_msgs::msg::Point leg_detected_[tab_size];
  int leg_cluster_[tab_size];
  bool leg_dynamic_[tab_size];

  // Person detection
  int nb_persons_detected_;
  geometry_msgs::msg::Point detected_person_[tab_size];
  int leg_left_[tab_size], leg_right_[tab_size];
  bool person_dynamic_[tab_size];

  // Tracking
  bool is_moving_person_detected_;
  geometry_msgs::msg::Point moving_detected_person_;
  bool is_person_tracked_, was_person_tracked_;
  geometry_msgs::msg::Point tracked_person_;
  bool associated_;
  int index_min_;
  int frequency_;
  float uncertainty_;

public:
  Datmo();

  void update();
  void update_state_variables();

  void store_background();
  void reset_motion();
  void detect_current_motion();
  void detect_motion();

  void perform_clustering(float clustering_threshold = default_cluster_threshold);
  void perform_basic_clustering(float clustering_threshold = default_cluster_threshold);
  void perform_advanced_clustering();
  int compute_nb_dynamic(int start, int end);

  void detect_legs();
  void detect_persons();
  void detect_a_moving_person();

  virtual void initialize_tracking();
  virtual void track_a_person();
  virtual void detect_and_track_a_person();

  void find_closest_obstacle();

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void robot_moving_callback(const std_msgs::msg::Bool::SharedPtr state);

  float distance_points(const geometry_msgs::msg::Point & pa, const geometry_msgs::msg::Point & pb);
  void save_data();

  void display_field_of_view();
  void display_motion();
  void display_clustering();
  void display_legs();
  void display_persons();
  void display_a_moving_detected_person();
  virtual void display_a_tracked_person();
  void display_a_detected_and_tracked_person();

};

#endif // DATMO_HPP
