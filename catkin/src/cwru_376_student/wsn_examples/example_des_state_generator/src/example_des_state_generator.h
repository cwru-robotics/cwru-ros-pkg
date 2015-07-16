// example_des_state_generator.h header file //
// wsn; Feb, 2015
// include this file in "example_des_state_generator.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef EXAMPLE_DES_STATE_GENERATOR_H_
#define EXAMPLE_DES_STATE_GENERATOR_H_

const bool DEBUG_MODE=false; // change this for display/break-points

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

#include <cwru_msgs/PathSegment.h>

//Eigen is useful for linear algebra
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

#include <tf/transform_listener.h> //for transforms

//Segment types 
const int HALT = 0;
const int LINE = cwru_msgs::PathSegment::LINE;
const int ARC = cwru_msgs::PathSegment::ARC;
const int SPIN_IN_PLACE = cwru_msgs::PathSegment::SPIN_IN_PLACE;

// dynamic limitations
const double MAX_SPEED = 0.5; // m/sec; adjust this
const double MAX_OMEGA = 0.5; //1.0; // rad/sec; adjust this
const double MAX_ACCEL = 1.0; // m/sec^2; adjust this
const double MAX_ALPHA = 1.0; // rad/sec^2; adjust this

const double LENGTH_TOL = 0.05; // tolerance for path; adjust this
const double HEADING_TOL = 0.05; // heading tolerance; adjust this

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
    bool get_current_path_seg_done() { return current_path_seg_done_; }     
    
    
    double min_dang(double dang); // compute periodic solution for smallest magnitude of angle of dang, e.g. +/- 2pi
    //a couple of utility functions: convert quaternion to heading and vice versa, for planar motion
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
    double compute_heading_from_v1_v2(Eigen::Vector2d v1, Eigen::Vector2d v2);

    geometry_msgs::PoseStamped map_to_odom_pose(geometry_msgs::PoseStamped map_pose);
    
    //geometry_msgs::Pose map_to_odom_pose(geometry_msgs::Pose map_pose); // convert a pose from map frame to odom frame
    geometry_msgs::PoseStamped odom_to_map_pose(geometry_msgs::PoseStamped odom_pose); // convert a pose from odom frame to map frame   


    //the interesting functions: how to get a new path segment and how to update the desired state
    void update_des_state();
    void unpack_next_path_segment();
 
    
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
    std::queue<cwru_msgs::PathSegment> segment_queue_; // path segment objects--as generated from crude polyline path (above)

    geometry_msgs::PoseStamped last_map_pose_rcvd_;
    geometry_msgs::Pose new_pose_des_;
    nav_msgs::Odometry des_state_;

    //state values from odometry; these will get filled in by odom callback
    nav_msgs::Odometry current_odom_;    
    geometry_msgs::Pose odom_pose_;    
    geometry_msgs::PoseStamped odom_pose_stamped_;     
    double odom_vel_;
    double odom_omega_;
    double odom_x_;
    double odom_y_;
    double odom_phi_;
    geometry_msgs::Quaternion odom_quat_;

    //path description values:  these are all with respect to odom coordinates
    // these values get set once upon construction of the current path segment:
    double current_seg_init_tan_angle_;  
    double current_seg_curvature_;
    double current_seg_length_; 
    int current_seg_type_; 
    double current_seg_phi_goal_;
    Eigen::Vector2d current_seg_tangent_vec_; 
    Eigen::Vector2d current_seg_ref_point_;
    
    // these values will evolve as desired state is updated:
    double current_seg_phi_des_;    
    double current_seg_length_to_go_;
    Eigen::Vector2d current_seg_xy_des_;
    double current_speed_des_;
    double current_omega_des_;
    bool current_path_seg_done_;
    
    
    bool waiting_for_vertex_;
    
    tf::TransformListener* tfListener_;
    tf::StampedTransform mapToOdom_;    
    tf::StampedTransform odomToMap_;    

    // PRIVATE METHODS:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();

    //prototypes for subscription callbacks
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    
    //prototypes for service callbacks 
    bool flushPathCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
    bool appendPathCallback(cwru_srv::path_service_messageRequest& request, cwru_srv::path_service_messageResponse& response);
 
    // function to draw a subgoal from the polyline path queue and re-interpret it in terms of multiple, dynamically-feasible
    // path segments
    void process_new_vertex();
        
    //construct path segments:
    // build_spin_then_line_path_segments: given two poses, p1 and p2 (in consistent reference frame),
    // construct a vector of path segments consistent with those poses;
    // for just two poses, command spin segment to reorient along path from p1 to p2, 
    // then a second segment to move along line from p1 to p2
    // So, this function will return a vector of path segments of size=2
    std::vector<cwru_msgs::PathSegment> build_spin_then_line_path_segments(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    // helper functions for the above: how to construct line and spin path segments
    cwru_msgs::PathSegment build_line_segment(Eigen::Vector2d v1, Eigen::Vector2d v2);
    cwru_msgs::PathSegment build_spin_in_place_segment(Eigen::Vector2d v1, double init_heading, double des_heading);
    // TODO: augment with circular-arc segments
    cwru_msgs::PathSegment build_arc_segment(Eigen::Vector2d arc_center, double init_heading, double final_heading, double curvature);
    
    //interpret path segments:
    // dissect a pathSegment object and fill in useful parameters as member vars;
    // also, initialize dynamic values in member vars
    void unpack_path_segment(cwru_msgs::PathSegment path_segment);
    
    // these should do triangular or trapezoidal velocity profiling
    // they should also be smart enough to recognize E-stops, etc.
    double compute_speed_profile();
    double compute_omega_profile();    
    
    // these are "crawler" functions.  Given a current path segment, they update desired state objects
    // and publish the resulting desired state;
    // When a segment is fully traversed, the segment type is set to HALT and the flag current_path_seg_done_ is set to true
    // the published desired state in the HALT condition is based on the last subgoal pose received, continuously updated
    // with the latest map_to_odom transform
    nav_msgs::Odometry update_des_state_lineseg();
    nav_msgs::Odometry update_des_state_spin();
    nav_msgs::Odometry update_des_state_halt();

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
