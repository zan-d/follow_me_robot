#include "datmo.hpp"

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::update() 
{
    if (!init_laser_) {
        previous_robot_moving_ = true;
        current_robot_moving_ = true;
    }
    
    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser_ && new_robot_ ) 
    {

        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "New data of laser received");
        RCLCPP_INFO(this->get_logger(), "New data of robot_moving received");

        // display field of view
        display_field_of_view();

        //detection of motion
        detect_motion();
        display_motion();

        // clustering
        perform_clustering(); // to perform clustering
        display_clustering();

        // detection of legs
        detect_legs(); // to detect legs using cluster
        display_legs();

        // detection of persons
        detect_persons(); // to detect persons using legs detected
        display_persons();
       
        detect_and_track_a_person();
        display_a_detected_and_tracked_person();

        update_state_variables();      

    }
    else
    {
        if ( !init_laser_ )
            RCLCPP_WARN(this->get_logger(), "waiting for laser data: run a rosbag");
        else
            if ( !init_robot_ )
            {
                RCLCPP_WARN(this->get_logger(), "waiting for robot_moving_node: ros2 run follow_me robot_moving_node");
                display_field_of_view();
            }
    }

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