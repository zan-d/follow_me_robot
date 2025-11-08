#include "datmo.hpp"

// DETECT MOTION FOR LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::store_background()
{
    // TO COMPLETE
    // store all the hits of the laser in the background table
    for (int loop_hit = 0; loop_hit < nb_beams_; loop_hit++)
    {
        // Display laser hit information in the terminal to understand the data structure and types
        RCLCPP_INFO(this->get_logger(),"r[%i] = %f, theta[%i] (in degrees) = %f, x[%i] = %f, y[%i] = %f",
                    loop_hit, r_[loop_hit], loop_hit, theta_[loop_hit]*180/M_PI, loop_hit, current_scan_[loop_hit].x, loop_hit, current_scan_[loop_hit].y);

        // Add laser hit to background table
        background_.pushback(r_[loop_hit],theta_[loop_hit]*180/M_PI);

    }

} // store_background

void Datmo::reset_motion()
{
    // TO COMPLETE
    //bool dynamic_reset_[tab_size];
    // for each hit, we reset the dynamic table
    for (int loop_hit = 0; loop_hit < nb_beams_; loop_hit++)
        {dynamic_[loop_hit]=0;}

} // reset_motion

void Datmo::detect_current_motion()
{
    // TO COMPLETE
    // for each hit, compare the current range with the background to detect motion
    // we fill the table dynamic
    for (int loop_hit = 0; loop_hit < nb_beams_; loop_hit++)
        {
            if background_[loop_hit][0]==r_[loop_hit] && background_[loop_hit][1]==theta_[loop_hit]
            {
                dynamic_[loop_hit]=0;
            }
            else{
                dynamic_[loop_hit]=1;
            }
        }

} // detect_simple_motion

void Datmo::detect_motion()
{
    // TO COMPLETE
    // you should integrate store_background, reset_motion and detect_current_motion in this function
    if (!current_robot_moving_)
    {
        // the robot is not moving then we can perform moving person detection
        // DO NOT FORGET to store the background but when ???
        if (!previous_robot_moving_)
        {
            // the robot was not moving
            detect_current_motion();
        }
        else
        {
            // the robot was moving
            store_background();
            reset_motion();
        }
    }
    else
    {
        // the robot is moving
        // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
        // what is the value of dynamic table for each hit of the laser ?
        if (!previous_robot_moving_)
        {
            // the robot was not moving
            reset_motion();
        }
        else
        {
            // the robot was moving
            reset_motion();
        }
    }

} // detect_motion

// CLUSTERING FOR LASER DATA
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::perform_clustering(float clustering_threshold)
{
    
    perform_basic_clustering(clustering_threshold);
    perform_advanced_clustering();

} // perform_clustering

void Datmo::perform_basic_clustering(float cluster_threshold)
{
    // TO COMPLETE
    // we perform the clustering as described in the lecture on perception
    // the data related to each cluster are stored in cluster_start, cluster_end and nb_cluster: see Datmo.h for more details
    // use: float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) to compute the euclidian distance between 2 points
    nb_clusters_ = 0;
    for (int loop_hit = 1; loop_hit < nb_beams_; loop_hit++)
    {
        // TO COMPLETE
        /*     if EUCLIDIAN DISTANCE between (the previous hit and the current one) is higher than "cluster_threshold"
                {//the current hit does not belong to the same cluster*/
                // to compute the euclidian distance use : float Datmo::distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
        
    }
     //Dont forget to update the different information for the last cluster
    //...

} // perform_basic_clustering

void Datmo::perform_advanced_clustering()
{
    // TO COMPLETE
    /* for each cluster, we update:
        - cluster_size to store the size of the cluster ie, the euclidian distance between the first hit of the cluster and the last one
        - cluster_middle to store the middle of the cluster
        - cluster_dynamic to store the percentage of hits of the current cluster that are dynamic
        - cluster_nb_points to store the number of hits in the cluster
        */
    // use: float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) to compute the euclidian distance between 2 points
    for (int loop_cluster = 0; loop_cluster < nb_clusters_; loop_cluster++)
    {
        int start = cluster_start_[loop_cluster];
        int end = cluster_end_[loop_cluster];
    }

} // perform_advanced_clustering

int Datmo::compute_nb_dynamic(int start, int end)
{
    //TO COMPLETE
    // return the number of points that are dynamic between start and end
    int nb_dynamic = 0;
 
    return (nb_dynamic);

} // compute_nb_dynamic

// DETECTION OF PERSONS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::detect_legs()
{
    // TO COMPLETE
    // a leg is a cluster:
        // - with a size higher than "leg_size_min";
        // - with a size lower than "leg_size_max;
        // - with more than 5 hits
        // if more than "dynamic_threshold"% of its hits are dynamic the leg is considered to be dynamic
        // we update the array leg_cluster, leg_detected and leg_dynamic
    nb_legs_detected_ = 0;
    for (int loop=0; loop<nb_clusters_; loop++)//loop over all the clusters
    {
        // TO COMPLETE
    }
   
} // detect_legs

void Datmo::detect_persons()
{
    //TO COMPLETE
    // a person has two legs located at less than "legs_distance_max" one from the other
    // a moving person (ie, person_dynamic array) has 2 legs that are dynamic
    // we update the detected_person table to store the middle of the person
    // we update the person_dynamic table to know if the person is moving or not      
    nb_persons_detected_ = 0;
    for (int loop_leg_left = 0; loop_leg_left < nb_legs_detected_; loop_leg_left++)
        for (int loop_leg_right = 1+loop_leg_left; loop_leg_right < nb_legs_detected_; loop_leg_right++)
        {
            // TO COMPLETE
        }
    
} // detect_persons

void Datmo::detect_a_moving_person()
{
    // TO COMPLETE for detection
    // we store the moving_detected_person in moving_detected_person
    // we update the boolean is_moving_detected_person
    
    is_moving_person_detected_ = false;
    for (int loop_persons = 0; loop_persons < nb_persons_detected_; loop_persons++)
        if (person_dynamic_[loop_persons])
        {
            //TO COMPLETE
        }
    /*if ( is_moving_person_detected )
        pub_detection.publish(moving_detected_person);*/

} // detect_a_moving_person

// TRACKING OF A PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::initialize_tracking()
{
    //Initialize tracking before running track_a_person();
}

void Datmo::track_a_person()
{
    //see slides 29 to 42 related to tracking in the Datmo lecture
    float distance_min;
    
    // association between the tracked person and the possible detection
    for (int loop_persons = 0; loop_persons < nb_persons_detected_; loop_persons++)
    {
        float current_dist;
        // we search for the detected_person which is the closest one to the tracked_person
        // we store the related information in index_min_, distance_min and associated_
    }
    
    if ( associated_ )
    {
        // update the information related to the tracked_person, frequency and uncertainty knowing that there is an association
        // should we publish or not tracked_person ?
    }
    else
    {
        // update the information related to the tracked_person, frequency and uncertainty knowing that there is no association
        // should we publish or not tracked_person ?
    }
    // do not forget to update tracked_person according to the current association
    
}

void Datmo::detect_and_track_a_person()
{

    //TO COMPLETE
    // When do we do detection and when do we do tracking ?
    // use the two boolean variables: is_moving_person_detected_ and is_person_tracked_ 
    // to check if you are detecting a moving person or tracking a person
    // the 3 methods you have to use are:
    // detect_a_moving_person(), initialize_tracking() and track_a_person();

}