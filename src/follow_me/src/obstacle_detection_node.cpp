#include "datmo.hpp"

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::update() 
{

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser_ ) 
    {

        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "New data of laser received");

        find_closest_obstacle();
    }
    else
        RCLCPP_WARN(this->get_logger(), "waiting for laser data: run a rosbag");

}// update

int main(int argc, char ** argv)
{
  // 1) Initialisation du contexte ROS 2
  rclcpp::init(argc, argv);

  // 2) Création de votre node datmo (le nom est défini dans le constructeur datmo())
  auto node = std::make_shared<Datmo>();

  // 3) Log d’attente
  RCLCPP_INFO(node->get_logger(), "waiting for detection of a moving person");

  // 4) Taux de boucle à 10 Hz
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