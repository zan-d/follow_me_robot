#include "action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

void Action::update()
{
    if (init_odom_ && cond_rotation_)
    {
        display_new_goal_to_reach();
        compute_rotation();
        display_rotation();
        move_robot();
        new_goal_to_reach_ = false;
    }
    else if (!init_odom_)
    {
        RCLCPP_WARN(this->get_logger(), "waiting for /odom");
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

  // 4) Boucle de traitement à action_node_hz Hz
  rclcpp::Rate loop_rate(action_node_hz); 

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);   // Gestion non bloquante des callbacks
    node->update();            // Méthode principale
    loop_rate.sleep();
  }

  // 5) Shutdown ROS proprement
  rclcpp::shutdown();
  return 0;
}
