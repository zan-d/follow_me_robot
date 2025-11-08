#pragma once

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>

#include <cmath>

inline constexpr int action_node_hz                 = 10; //With UST15LX lidar, we can go up to 40hz. Make sure other datmo node hz matches.
inline constexpr float translation_speed_max        = 0.6f; //Hardware max: 0.8m/s
inline constexpr float translation_accel_max        = 0.8f; //Hardware max: 2.667m/s^2
inline constexpr float rotation_speed_max           = static_cast<float>(M_PI); // previous: static_cast<float>(M_PI / 3.0); // Hardware max 4.4rad/s
inline constexpr float rotation_accel_max           = static_cast<float>(M_PI*2); //Hardware max 13.0rad/s^2
inline constexpr float rotation_coefficient_max     = static_cast<float>(M_PI / 6.0);

inline constexpr float error_translation_threshold  = 0.1f; // 0.3f
inline constexpr float safety_distance              = 0.35f; //0.2f
inline constexpr float person_radius                = 0.3f;
inline constexpr float error_rotation_threshold     = static_cast<float>(M_PI / 9.0);
inline constexpr int nb_static                      = 5;


class Action : public rclcpp::Node
{
public:
    Action();

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_to_reach_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_obstacle_detection_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_stamped_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_robot_moving_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Goal
    geometry_msgs::msg::Point goal_to_reach_;
    bool new_goal_to_reach_ = false;

    // Rotation PID
    float rotation_to_do_, rotation_done_;
    bool cond_rotation_;
    float initial_orientation_, current_orientation_, not_moving_orientation_;
    float error_rotation_;
    float error_integral_rotation_;
    float error_previous_rotation_;
    float current_rotation_speed_;
    float previous_rotation_speed_;
    float kpr_, kir_, kdr_;

    // Translation PID
    float translation_to_do_, translation_done_;
    bool cond_translation_;
    geometry_msgs::msg::Point initial_position_, current_position_, not_moving_position_;
    float error_translation_;
    float error_integral_translation_;
    float error_previous_translation_;
    float current_translation_speed_;
    float previous_translation_speed_;
    float kpt_, kit_, kdt_;

    // Obstacle
    bool init_odom_ = false;
    bool init_obstacle_detected_ = false;
    bool new_obstacle_detected_ = false;
    geometry_msgs::msg::Point obstacle_detected_;

    float coef_rotation_, coef_translation_;

    bool robot_moving_;

public:
    // Update methods
    void update();
    void compute_rotation(bool stop = true);
    void compute_translation();
    void combine_rotation_and_translation();
    void move_robot();
    void detect_obstacle();
    void check_if_robot_is_moving();

    // Display methods (log)
    void display_new_goal_to_reach();
    void display_rotation();
    void display_translation();
    void display_obstacle();
    void display_rotation_and_translation();
    void display_robot_moving();

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goal_to_reach_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void obstacle_callback(const geometry_msgs::msg::Point::SharedPtr msg);

    // Utils
    float distance_points(const geometry_msgs::msg::Point &pa, const geometry_msgs::msg::Point &pb) const;
};
