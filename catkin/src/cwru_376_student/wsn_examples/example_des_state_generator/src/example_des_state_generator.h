// example_des_state_generator.h header file //
// wsn; Feb, 2015
// include this file in "example_des_state_generator.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef EXAMPLE_DES_STATE_GENERATOR_H_
#define EXAMPLE_DES_STATE_GENERATOR_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

//Eigen is useful for linear algebra
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

#include <tf/transform_listener.h> //for transforms

//Segment types 
const int HALT = 0;
const int LINE = 1;
const int ARC = 2;
const int SPIN_IN_PLACE = 3;

// dynamic limitations
const double MAX_SPEED = 0.1; // m/sec; adjust this
const double MAX_OMEGA = 0.1; //1.0; // rad/sec; adjust this
const double MAX_ACCEL = 1.0; // m/sec^2; adjust this
const double MAX_ALPHA = 1.0; // rad/sec^2; adjust this

const double LENGTH_TOL = 0.05; // tolerance for path; adjust this
const double HEADING_TOL = 0.1; // heading tolerance; adjust this

const double UPDATE_RATE = 50.0; // choose the desired-state publication update rate

// define a class, including a constructor, member variables and member functions

class DesStateGenerator {
public:
    // PUBLIC MEMBER FUNCTIONS:
    DesStateGenerator(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor

    // some utilities:
    //signum function: define this one in-line
    double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
    }
    
    bool get_waiting_for_vertex() { return waiting_for_vertex_; }  
    
    double min_dang(double dang); // compute periodic solution for smallest magnitude of angle of dang, e.g. +/- 2pi
    //a couple of utility functions: convert quaternion to heading and vice versa, for planar motion
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

    geometry_msgs::Pose map_to_odom_pose(geometry_msgs::Pose map_pose); // convert a pose from map frame to odom frame
    geometry_msgs::Pose odom_to_map_pose(geometry_msgs::Pose odom_pose); // convert a pose from odom frame to map frame   


    //the interesting functions: how to get a new vertex and compute path segment properties, and how to update the desired state
    void process_new_vertex();
    void update_des_state();
 
    
private:
    
    //PRIVATE DATA:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber odom_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::ServiceServer append_path_; // service to receive a path message and append the poses to a queue of poses
    ros::ServiceServer flush_path_; //service to clear out the current queue of path points
    ros::Publisher des_state_publisher_; // we will publish desired states using this object   

    double dt_; // time step of update rate
    std::queue<geometry_msgs::PoseStamped> path_queue_; //a C++ "queue" object, stores vertices as Pose points in a FIFO queue; receive these via appendPath service
    geometry_msgs::Pose last_map_pose_rcvd_;
    geometry_msgs::Pose odom_pose_;
    geometry_msgs::Pose new_pose_des_;
    nav_msgs::Odometry current_odom_;
    nav_msgs::Odometry des_state_;

    //state values from odometry; these will get filled in by odom callback
    double odom_vel_;
    double odom_omega_;
    double odom_x_;
    double odom_y_;
    double odom_phi_;

    //path description values:  these are all with respect to odom coordinates
    Eigen::Vector2d v1_, v2_, v3_; // 3 path vertices, 2-D vectors of type "Eigen"; should augment this for arcs
    double phi1_, phi2_, phi3_; // corresponding heading values
    double phi_path_des_;
    double phi_goal_;
    Eigen::Vector2d tangent_vec_;
    double spin_dir_;
    double current_segment_length_; // length of current segment--could be meters or radians, depending on segment type
    double current_segment_length_to_go_; // remaining distance/angle to be covered for this segment
    int current_segment_type_; //line, arc or spin-in-place, per definition codes above
    bool waiting_for_vertex_;

    // PRIVATE METHODS:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();

    //prototypes for subscription callbacks
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    
    //prototypes for service callbacks 
    bool flushPathCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
    bool appendPathCallback(cwru_srv::path_service_messageRequest& request, cwru_srv::path_service_messageResponse& response);
 
    
    void build_path_segment(geometry_msgs::Pose map_pose);
    void build_line_segment();
    void build_spin_in_place_segment();
    
    double compute_speed_profile();
    double compute_omega_profile();    
    

    nav_msgs::Odometry update_des_state_lineseg();
    nav_msgs::Odometry update_des_state_spin();
    nav_msgs::Odometry update_des_state_halt();

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
