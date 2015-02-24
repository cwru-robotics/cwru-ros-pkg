//example_des_generator.cpp:
//wsn, Feb 2015
//implementation of class to generate a stream of desired states

// can test the flush-path service manually with 
// rosservice call flushPathService 1

// can test service appendPathCallback using test node: example_path_sender.cpp


// this header incorporates all the necessary #include files and defines the class "DesStateGenerator"
#include "example_des_state_generator.h"
int ans;

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc

DesStateGenerator::DesStateGenerator(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { // constructor
    ROS_INFO("in class constructor of DesStateGenerator");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();

    odom_phi_ = 1000.0; // put in impossible value for heading; test this value to make sure we have received a viable odom message
    ROS_INFO("waiting for valid odom message...");
    while (odom_phi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("constructor: got an odom message");
    
    dt_ = 1.0/UPDATE_RATE; // time step consistent with update frequency
    //initialize variables here, as needed
    //for start-up, use the current odom values for the first vertex of the path
    v1_(0) = odom_x_;
    v1_(1) = odom_y_;
    phi1_ = odom_phi_;
    phi_path_des_ = phi1_;
    v2_ = v1_;
    phi2_ = phi1_;

    current_segment_length_ = 0.0;
    current_segment_length_to_go_ = -0.001; // declare that current segment is complete
    current_segment_type_ = HALT; // special case--no plan to proceed from here
    waiting_for_vertex_ = true;
    new_pose_des_ = odom_pose_; // init most current goal to current odom pose
    last_map_pose_rcvd_ = odom_to_map_pose(odom_pose_); // treat the current odom pose as the first vertex--cast it into map coords to save
}


//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass

void DesStateGenerator::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    odom_subscriber_ = nh_.subscribe("/odom", 1, &DesStateGenerator::odomCallback, this); //subscribe to odom messages
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"

void DesStateGenerator::initializeServices() {
    ROS_INFO("Initializing Services");
    flush_path_ = nh_.advertiseService("flushPathService",
            &DesStateGenerator::flushPathCallback,
            this);
    append_path_ = nh_.advertiseService("appendPathService",
            &DesStateGenerator::appendPathCallback,
            this);
    // add more services here, as needed
}

//member helper function to set up publishers;
void DesStateGenerator::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    des_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("desState", 1, true); // publish des state in same format as odometry messages
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}


void DesStateGenerator::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd;
    odom_pose_ = odom_rcvd.pose.pose;
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_rcvd.pose.pose.orientation); // cheap conversion from quaternion to heading for planar motion
}

//member function implementation for a service callback function
bool DesStateGenerator::flushPathCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service flush-Path callback activated");
    while (!path_queue_.empty()) {
        ROS_INFO("clearing the path queue...");
        std::cout << ' ' << path_queue_.front();
        path_queue_.pop();
    }
    response.resp = true; // boring, but valid response info
    return true;
}

bool DesStateGenerator::appendPathCallback(cwru_srv::path_service_messageRequest& request, cwru_srv::path_service_messageResponse& response) {
    geometry_msgs::PoseStamped pose;
    double x, y, phi;
    geometry_msgs::Quaternion quaternion;
    ROS_INFO("service append-Path callback activated");
    /* Path message:
     * #An array of poses that represents a Path for a robot to follow
        Header header
        geometry_msgs/PoseStamped[] poses
     */
    int nposes = request.path.poses.size();
    ROS_INFO("received %d vertices", nposes);
    for (int ipose = 0; ipose < nposes; ipose++) {
        ROS_INFO("pushing a pose onto queue");
        pose = request.path.poses[ipose];
        x = pose.pose.position.x;
        y = pose.pose.position.y;
        quaternion = pose.pose.orientation;
        phi = convertPlanarQuat2Phi(quaternion);
        std::cout << "x,y,phi = " << x << ", " << y << ", " << phi << std::endl;
        path_queue_.push(pose);
    }
    int nqueue = path_queue_.size();
    ROS_INFO("queue now contains %d vertices", nqueue);
    response.resp = true; // boring, but valid response info
    return true;
}

//some conversion utilities:
double DesStateGenerator::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

geometry_msgs::Quaternion DesStateGenerator::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

//utility fnc to compute min dang, accounting for periodicity
double DesStateGenerator::min_dang(double dang) {
    if (dang > M_PI) dang -= 2.0 * M_PI;
    if (dang<-M_PI) dang += 2.0 * M_PI;
    return dang;
}

//DUMMY...
geometry_msgs::Pose DesStateGenerator::map_to_odom_pose(geometry_msgs::Pose map_pose) {
    return map_pose; // dummy--no conversion; when AMCL is running, use base-frame transform to convert from map to odom coords
}

//DUMMY...
geometry_msgs::Pose DesStateGenerator::odom_to_map_pose(geometry_msgs::Pose odom_pose) {
    return odom_pose; // dummy--no conversion; when AMCL is running, use base-frame transform to convert from map to odom coords
}

void DesStateGenerator::process_new_vertex() {
    if (path_queue_.empty()) { // do nothing
        waiting_for_vertex_ = true;
        current_segment_type_ = HALT;
        return;
    }
    
    ROS_INFO("process_new_vertex: ");
    int npts = path_queue_.size();
    ROS_INFO("there are %d vertices in the queue", npts);
    //if here, get the next vertex from the queue, convert to odom coords, and set up path segment params
    waiting_for_vertex_ = false; //will build new path segments from most recent path vertex
    geometry_msgs::PoseStamped map_pose_stamped = path_queue_.front(); // note: we have a copy of front of queue, but we have not popped it from the queue yet
    geometry_msgs::Pose map_pose = map_pose_stamped.pose; //strip off the header
    last_map_pose_rcvd_ = map_pose; // store this in memory
    
    // lots of work here...; spin in place to point to new vertex, then move to new vertex, then pop it from the queue
    build_path_segment(map_pose); // convert from map coords to find vertex and heading in odom coords     

}


//have a new goal vertex (in map coords); construct new path segments from this (spin, then translate)
// clever (ugly?) trick: first treat this goal for reorientation segment, but don't pop it from the queue
// process_new_vertex() will grab the same goal twice--and the second time, we will build a line segment path

void DesStateGenerator::build_path_segment(geometry_msgs::Pose map_pose) {
    geometry_msgs::Pose des_pose_wrt_odom;
    des_pose_wrt_odom = map_to_odom_pose(map_pose); // convert new vertex pose from map to odom coords

    // choice: re-use last vertex, or use latest odom coords
    // start from current odometry values
    //v1_(0) = odom_x_;
    //v1_(1) = odom_y_;
    //phi1_ = odom_phi_;
    v1_ = v2_; // this re-uses previous subgoal as new starting point
    phi1_ = phi_goal_;

    // pick out the values we need for new goal pose 
    v2_(0) = des_pose_wrt_odom.position.x;
    v2_(1) = des_pose_wrt_odom.position.y;  
    
    //and compute some move properties:
    Eigen::Vector2d dv = v2_ - v1_; //vector from v1 to v2
    double phi_12 = atan2(dv(1), dv(0)); //heading from v1 to v2
    double dist_12 = dv.norm(); // distance from v1 to v2

    // are we already at v2?
    if (dist_12 < LENGTH_TOL) {
        //v2 and v1 are essentially coincident; we are done with v2, so pop it from the queue and seek a new subgoal
        ROS_INFO("new vertex is approx coincident with current pose; ready for next vertex");
        std::cout<<"enter 1: ";
        std::cin>>ans;        
        waiting_for_vertex_ = true;
        current_segment_type_ = HALT;
        path_queue_.pop(); // throw away this vertex; we are done with it; attempt to get another vertex next iteration
        return;
    }

    // if here, we have not reached v2; we should: A) point towards v2, or B) drive towards v2, 
    // do we need to reorient to point to v2, or can we proceed towards v2?  check heading
    // phi_path_des_ is updated by spin-in-place move; test if this has converged on goal heading
    double dphi_12 = min_dang(phi_12 - phi_path_des_); //heading error from current odom heading to heading that points from v1 to v2
    if (fabs(dphi_12) > HEADING_TOL) { //SPIN: set up move params for a SPIN_IN_PLACE move
        build_spin_in_place_segment();
        return;
    }

    else { // final case: move along line segment
    // if here, we are oriented towards v2, but we need to get there: move along a line
    //prepare parameters for a line-segment move:
        build_line_segment();
        return;
    }
}

void DesStateGenerator::build_spin_in_place_segment() {
        //orient towards v2:
        ROS_INFO("build_spin_in_place_segment");
        Eigen::Vector2d dv = v2_ - v1_; //vector from v1 to v2; yeah, I know--was computed above
        phi_goal_ = atan2(dv(1), dv(0)); //heading from v1 to v2= target heading; head here incrementally
        spin_dir_ = sgn(phi_goal_- phi1_); // choose to spin in this direction
        current_segment_length_ = fabs(phi_goal_ - phi1_); // define segment length as magnitude of angular rotation to perform
        current_segment_length_to_go_ = current_segment_length_; // initialize distance to go as full length; magnitude only
        current_segment_type_ = SPIN_IN_PLACE;
        ROS_INFO("seg_length = %f",current_segment_length_);
        std::cout<<"enter 1: ";
        std::cin>>ans;        
        return;
}

void DesStateGenerator::build_line_segment() {
        //orient towards v2:
        ROS_INFO("build_line_segment");
        Eigen::Vector2d dv = v2_ - v1_; //vector from v1 to v2        
        tangent_vec_ = dv/(dv.norm()); // unit vector from v1 to v2;   division by 0 should not be a problem, since current_segment_length_ > LENGTH_TOL 
        phi_goal_ = atan2(dv(1), dv(0)); //heading from v1 to v2= target heading; head here incrementally
        current_segment_length_ = dv.norm(); // 
        current_segment_length_to_go_ = current_segment_length_; // initialize distance-to-go = full length;                                      
        current_segment_type_ = LINE;

        ROS_INFO("seg_length = %f",current_segment_length_);
        std::cout<<"enter 1: ";
        std::cin>>ans;        
        return;
}

void DesStateGenerator::update_des_state() {
    if (current_segment_type_ == LINE) {
        des_state_ = update_des_state_lineseg();
    } else if (current_segment_type_ == SPIN_IN_PLACE) {
        des_state_ = update_des_state_spin();
    } else { // this should be the HALT case:
        des_state_ = update_des_state_halt();
        des_state_.header.stamp = ros::Time::now();
        new_pose_des_ = map_to_odom_pose(last_map_pose_rcvd_);
        des_state_.pose.pose = new_pose_des_;
    }
    des_state_publisher_.publish(des_state_); //send out our message
}


// NEED TO WRITE THESE... means to update the desired state incrementally, given path segment params
// and dynamic limits on vel and accel
// need to identify when current segment is complete

//update translational desired state along a line segment from v1 to v2
nav_msgs::Odometry DesStateGenerator::update_des_state_lineseg() {
    nav_msgs::Odometry desired_state; // fill in this message and return it
    Eigen::Vector2d xy_des;
    double omega_des = 0.0;
    
    double speed_des = compute_speed_profile(); //USE VEL PROFILING

    double delta_s = speed_des*dt_; //incremental forward move distance; a scalar
    
    current_segment_length_to_go_ -= delta_s; // plan to move forward by this much
    ROS_INFO("update_des_state_lineseg: current_segment_length_to_go_ = %f",current_segment_length_to_go_);     
    if (current_segment_length_to_go_ < LENGTH_TOL) { // check if done with this move
        // done with line segment;
        current_segment_type_ = HALT;
        xy_des = v2_; // specify destination vertex as exact, current goal
        current_segment_length_to_go_=0.0;
        speed_des = 0.0;  // 
        waiting_for_vertex_ = true;
        ROS_INFO("update_des_state_spin: done with spin");
    }
    else { // not done yet--move forward
        // based on distance covered, compute current desired x,y; use scaled vector from v1 to v2 
        xy_des = v1_ + tangent_vec_*(current_segment_length_ - current_segment_length_to_go_);   
        std::cout<<"tan vec and xy_des: "<<std::endl;
        std::cout<<tangent_vec_<<std::endl;
        std::cout<<xy_des<<std::endl;
    }

    // fill in components of desired-state message:
    desired_state.twist.twist.linear.x =speed_des;
    desired_state.twist.twist.angular.z = omega_des;
    desired_state.pose.pose.position.x = xy_des(0);
    desired_state.pose.pose.position.y = xy_des(1);
    desired_state.pose.pose.orientation = convertPlanarPhi2Quaternion(phi_goal_);
    desired_state.header.stamp = ros::Time::now();
    return desired_state; 
}



nav_msgs::Odometry DesStateGenerator::update_des_state_spin() {
    nav_msgs::Odometry desired_state; // fill in this message and return it
    Eigen::Vector2d xy_des;
    double omega_des = compute_omega_profile(); //USE VEL PROFILING
    double speed_des = 0.0; 

    double delta_phi = omega_des*dt_; //incremental rotation--could be + or -
    xy_des = v1_; // specify starting vertex as exact, current desired location
    ROS_INFO("update_des_state_spin: delta_phi = %f",delta_phi);
    current_segment_length_to_go_ -= fabs(delta_phi); // decrement the (absolute) distance (rotation) to go
    ROS_INFO("update_des_state_spin: current_segment_length_to_go_ = %f",current_segment_length_to_go_);    
    if (current_segment_length_to_go_ < HEADING_TOL) { // check if done with this move
        // done with line segment;
        current_segment_type_ = HALT;

        current_segment_length_to_go_=0.0;
        speed_des = 0.0;  // 
        omega_des = 0.0;
        phi_path_des_ = phi_goal_;
        waiting_for_vertex_ = true; 
        ROS_INFO("update_des_state_spin: done with spin");
    }
    else { // not done yet--move forward
        // based on distance covered, compute current desired x,y; use scaled vector from v1 to v2 
        phi_path_des_ = phi1_ + spin_dir_*(current_segment_length_ - current_segment_length_to_go_);       
    }
    
    // fill in components of desired-state message:
    desired_state.twist.twist.linear.x =speed_des;
    desired_state.twist.twist.angular.z = omega_des;
    desired_state.pose.pose.position.x = xy_des(0);
    desired_state.pose.pose.position.y = xy_des(1);
    desired_state.pose.pose.orientation = convertPlanarPhi2Quaternion(phi_path_des_);
    desired_state.header.stamp = ros::Time::now();
    return desired_state;         
}

nav_msgs::Odometry DesStateGenerator::update_des_state_halt() {
    nav_msgs::Odometry desired_state; // fill in this message and return it
    // fill in components of desired-state message from most recent odom message
    desired_state = current_odom_;
    desired_state.twist.twist.linear.x = 0.0; // but specified desired twist = 0.0
    desired_state.twist.twist.angular.z = 0.0;
    desired_state.header.stamp = ros::Time::now();
    return desired_state;         
}

//DUMMY--fill this in
double DesStateGenerator::compute_speed_profile() {
    return MAX_SPEED;
}

// MAKE THIS BETTER!!
double DesStateGenerator::compute_omega_profile() {
    double des_omega = sgn(spin_dir_)*MAX_OMEGA;
    ROS_INFO("compute_omega_profile: des_omega = %f",des_omega);
    return des_omega; // spin in direction of closest rotation to target heading
}


int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "desStateGenerator"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating a DesStateGenerator");
    DesStateGenerator desStateGenerator(&nh); //instantiate a DesStateGenerator object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate, e.g. 50Hz
    

    //constructor will wait for a valid odom message; let's use this for our first vertex;


    ROS_INFO("main: going into main loop");


    while (ros::ok()) {
        if (desStateGenerator.get_waiting_for_vertex()) {
            desStateGenerator.process_new_vertex(); //if possible, get a new vertex and compute segment requirements
        }

        desStateGenerator.update_des_state(); // update the desired state and publish it; when reach v2, set waiting_for_vertex_

        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
}

