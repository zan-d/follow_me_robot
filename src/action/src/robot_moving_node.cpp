#include "action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

void Action::update()
{
    if (init_odom_)
    {
        check_if_robot_is_moving();
        display_robot_moving();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "waiting for odometry");
    }
}

int main(int argc, char **argv)
{
  // 1) Initialisation du contexte ROS 2
  rclcpp::init(argc, argv);

  // 2) Création du nœud (Action doit hériter de rclcpp::Node)
  auto node = std::make_shared<Action>();  // ⚠️ Vérifie que la classe s'appelle bien `Action` avec un A majuscule

  // 3) Message d’information
  RCLCPP_INFO(node->get_logger(), "Waiting for detection of a moving person");

  // 4) Boucle de traitement à 10 Hz
  rclcpp::Rate loop_rate(10);  // 10 Hz

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);   // Gestion non bloquante des callbacks
    node->update();            // Méthode principale
    loop_rate.sleep();
  }

  // 5) Shutdown ROS proprement
  rclcpp::shutdown();
  return 0;
}
