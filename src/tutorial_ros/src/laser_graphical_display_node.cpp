// laser graphical display
// written by O. Aycard
// converted to ROS2 by P. Scales

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

#include <functional>  // pour std::bind
using std::placeholders::_1;

inline constexpr int tab_size = 1100;

class laser_graphical_display_node : public rclcpp::Node
{
protected:
  // ROS subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

  // Visualization markers
  visualization_msgs::msg::Marker marker_field_of_view_, marker_laser_;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_field_of_view_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_laser_graphical_display_marker_;
  
    // Laser scan data
    int nb_beams_;
    bool init_laser_, new_laser_; //to check if new data of laser is available or not
    float range_min_, range_max_;
    float angle_min_, angle_max_, angle_inc_;
    float r_[tab_size], theta_[tab_size];
    geometry_msgs::msg::Point current_scan_[tab_size];


public:
  laser_graphical_display_node();

  void update();

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  float distance_points(const geometry_msgs::msg::Point & pa, const geometry_msgs::msg::Point & pb);

  void display_field_of_view();

};



laser_graphical_display_node::laser_graphical_display_node()
: Node("laser_graphical_display_node")
{
  auto qos = rclcpp::SensorDataQoS();

  // Subscribers
  // Preparing a subscriber to the "scan" topic, in order to receive data from the laser scanner.
  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", qos, std::bind(&laser_graphical_display_node::scan_callback, this, _1));

  // Publishers
  // Preparing a topic to publish our results. This will be used by the visualization tool rviz
  pub_laser_graphical_display_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("laser_graphical_display_marker", 1);
  pub_field_of_view_marker_           = this->create_publisher<visualization_msgs::msg::Marker>("field_of_view_marker", 1);


  // Initialize state
  new_laser_ = false;
}


//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void laser_graphical_display_node::update() {
    // If we have received a new laser scan at this node cycle, process it.
    if ( new_laser_ )
    {
        new_laser_ = false;

        RCLCPP_INFO(this->get_logger(), "\n\n New data of laser received");

        std_msgs::msg::ColorRGBA color;
        marker_laser_.points.clear();
        marker_laser_.colors.clear();

        // Process the laser data
        for (int loop_hit = 0; loop_hit < nb_beams_; loop_hit++)
        {
            // Display laser hit information in the terminal to understand the data structure and types
            RCLCPP_INFO(this->get_logger(),"r[%i] = %f, theta[%i] (in degrees) = %f, x[%i] = %f, y[%i] = %f",
                        loop_hit, r_[loop_hit], loop_hit, theta_[loop_hit]*180/M_PI, loop_hit, current_scan_[loop_hit].x, loop_hit, current_scan_[loop_hit].y);

            // Add a marker based on the position of the laser hits in current_scan_
            marker_laser_.points.push_back(current_scan_[loop_hit]);

            // Determine the color of the marker
            color.r = 0; color.g = 0; color.b = 1.0; color.a = 1.0;
            marker_laser_.colors.push_back(color);
        }

        // Add necessary information required to plot the marker points on the Rviz graphical interface
        marker_laser_.header.frame_id = "laser";
        marker_laser_.header.stamp = this->get_clock()->now();
        marker_laser_.ns = "laser_graphical_display_marker";
        marker_laser_.id = 0;
        marker_laser_.type = visualization_msgs::msg::Marker::POINTS;
        marker_laser_.action = visualization_msgs::msg::Marker::ADD;
        marker_laser_.pose.orientation.w = 1;
        marker_laser_.scale.x = 0.05;
        marker_laser_.scale.y = 0.05;
        marker_laser_.color.a = 1.0;


        // Publish the marker_ message using the Publisher, making it available to other nodes, such as Rviz
        pub_laser_graphical_display_marker_->publish(marker_laser_);

        // Publish markers showing the lidar's field of view
        display_field_of_view();
    }
}// update

// CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void laser_graphical_display_node::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{

    init_laser_ = true;
    new_laser_ = true;
    
    // store the important data related to laserscanner
    range_min_ = scan->range_min;
    range_max_ = scan->range_max;
    angle_min_ = scan->angle_min;
    angle_max_ = scan->angle_max;
    angle_inc_ = scan->angle_increment;
    nb_beams_ = ((-1 * angle_min_) + angle_max_) / angle_inc_;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min_;
    for (int loop = 0; loop < nb_beams_; loop++, beam_angle += angle_inc_)
    {
        if ((scan->ranges[loop] < range_max_) && (scan->ranges[loop] > range_min_))
            r_[loop] = scan->ranges[loop];
        else
            r_[loop] = range_max_;
        theta_[loop] = beam_angle;

        // transform the scan in cartesian framewrok
        current_scan_[loop].x = r_[loop] * cos(beam_angle);
        current_scan_[loop].y = r_[loop] * sin(beam_angle);
        current_scan_[loop].z = 0.0;
        //ROS_INFO("laser[%i]: (%.2f, %.2f) -> (%.2f, %.2f)", loop, r[loop], theta[loop]*180/M_PI, current_scan_[loop].x, current_scan_[loop].y);
    }

} // scanCallback


// Implémentation de display_field_of_view
void laser_graphical_display_node::display_field_of_view()
{

  //  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "displaying field of view");

  visualization_msgs::msg::Marker references;
  references.header.frame_id = "laser";
  references.header.stamp = this->get_clock()->now();
  references.ns = "reference_marker";
  references.id = 1;
  references.type = visualization_msgs::msg::Marker::LINE_STRIP;
  references.action = visualization_msgs::msg::Marker::ADD;
  references.pose.orientation.w = 1.0;

  references.scale.x = 0.02;

  references.color.r = 1.0f;
  references.color.g = 1.0f;
  references.color.b = 1.0f;
  references.color.a = 1.0f;

  geometry_msgs::msg::Point p;
  // Origine
  p.x = 0.0;
  p.y = 0.0;
  p.z = 0.0;
  references.points.push_back(p);

  // Construction des bords du champ de vue
  float beam_angle = angle_min_;
  for (size_t i = 0; i < nb_beams_; ++i) {
      p.x = range_max_ * std::cos(beam_angle);
      p.y = range_max_ * std::sin(beam_angle);
      p.z = 0.0;
      references.points.push_back(p);
    beam_angle += angle_inc_;
  }

  // Retour à l'origine
  p.x = 0.0;
  p.y = 0.0;
  p.z = 0.0;
  references.points.push_back(p);

  // Publication du marker
  pub_field_of_view_marker_->publish(references);

  RCLCPP_INFO(this->get_logger(), "field of view displayed");
}



int main(int argc, char ** argv)
{
  // 1) Initialisation du contexte ROS 2
  rclcpp::init(argc, argv);

  // 2) Création de votre node laser_graphical_display_node (le nom est défini dans le constructeur laser_graphical_display_node())
  auto node = std::make_shared<laser_graphical_display_node>();

  // 3) Log d’attente
  RCLCPP_INFO(node->get_logger(), "waiting for laser data");

  // 4) Taux de boucle à 10Hz
  rclcpp::Rate rate(10 /*Hz*/);

  // 5) Boucle principale : on tourne les callbacks et on appelle update()
  while (rclcpp::ok()) {
    // Exécute une itération de callback sans bloquer
    rclcpp::spin_some(node);
    // Votre méthode d’update périodique
    node->update();
    rate.sleep();
  }

  // 6) Nettoyage et shutdown
  rclcpp::shutdown();
  return 0;
}