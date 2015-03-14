//example_tf_listener.cpp:
//wsn, March 2015
//illustrative node to show use of tf listener
// assumes use of a robot with a base_frame, and a map (e.g., from AMCL)



// this header incorporates all the necessary #include files and defines the class "SteeringController"


#include "example_tf_listener.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
DemoTfListener::DemoTfListener(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of DemoTfListener");
    tfListener_ = new tf::TransformListener;  //create a transform listener
    
    // wait to start receiving valid tf transforms between map and odom:
    bool tferr=true;
    ROS_INFO("waiting for tf between map and odom...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener_->lookupTransform("odom", "map", ros::Time(0), mapToOdom_);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    // from now on, tfListener will keep track of transforms
    
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
 
    
    //initialize desired state, in case this is not yet being published adequately
    des_state_ = current_odom_;  // use the current odom state
    // but make sure the speed/spin commands are set to zero
    current_speed_des_ = 0.0;  // 
    current_omega_des_ = 0.0;    
    des_state_.twist.twist.linear.x = current_speed_des_; // but specified desired twist = 0.0
    des_state_.twist.twist.angular.z = current_omega_des_;
    des_state_.header.stamp = ros::Time::now();   

    //initialize the twist command components, all to zero
    twist_cmd_.linear.x = 0.0;
    twist_cmd_.linear.y = 0.0;
    twist_cmd_.linear.z = 0.0;
    twist_cmd_.angular.x = 0.0;
    twist_cmd_.angular.y = 0.0;
    twist_cmd_.angular.z = 0.0;

    twist_cmd2_.twist = twist_cmd_; // copy the twist command into twist2 message
    twist_cmd2_.header.stamp = ros::Time::now(); // look up the time and put it in the header  

 
}

//member helper function to set up subscribers;
void DemoTfListener::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers: odom and desState");
    odom_subscriber_ = nh_.subscribe("/odom", 1, &DemoTfListener::odomCallback, this); //subscribe to odom messages
    // add more subscribers here, as needed

}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void DemoTfListener::initializeServices()
{
    ROS_INFO("Initializing Services: exampleMinimalService");
    //simple_service_ = nh_.advertiseService("exampleMinimalService",
    //                                               &SteeringController::serviceCallback,
    //                                               this);  
    // add more services here, as needed
}

//member helper function to set up publishers;
void DemoTfListener::initializePublishers()
{
    //ROS_INFO("Initializing Publishers: cmd_vel and cmd_vel_stamped");
    //cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true); // talks to the robot!
    //cmd_publisher2_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1, true); //alt topic, includes time stamp
    //steering_errs_publisher_ =  nh_.advertise<std_msgs::Float32MultiArray>("steering_errs",1, true);
}



void DemoTfListener::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    odom_pose_ = odom_rcvd.pose.pose;
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion
    
    // test for map to odom transforms:
    // Fill transform with the transform from source_frame to target_frame 
    // first arg, target frame, 2nd arg, source frame
    // e.g.,  I want to know where is the base_link w/rt the map: 
    //The direction of the transform returned will be from the target_frame to the source_frame. 
    // Which if applied to data, will transform data in the source_frame into the target_frame.
    // so, "where is the robot w/rt the map" can be found via:
    tfListener_->lookupTransform("map", "base_link", ros::Time(0), baseLink_wrt_map_);
    ROS_INFO("base link w/rt map: ");
    tf::Vector3 pos = baseLink_wrt_map_.getOrigin();
    tf::Quaternion tf_quaternion = baseLink_wrt_map_.getRotation();
    geometry_msgs::Quaternion quaternion;
    quaternion.x = tf_quaternion.x();
    quaternion.y = tf_quaternion.y();
    quaternion.z = tf_quaternion.z();
    quaternion.w = tf_quaternion.w();
    double yaw = convertPlanarQuat2Phi(quaternion);
    ROS_INFO("x,y, yaw = %f, %f, %f",pos[0],pos[1],yaw);
    ROS_INFO("q x,y,z,w: %f %f %f %f",quaternion.x,quaternion.y,quaternion.z,quaternion.w);
    
    //mapToOdom_
    //std::cout<<mapToOdom_<<std::endl;
    
}


//utility fnc to compute min dang, accounting for periodicity
double DemoTfListener::min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}


// saturation function, values -1 to 1
double DemoTfListener::sat(double x) {
    if (x>1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}

//some conversion utilities:
double DemoTfListener::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//member function implementation for a service callback function
// could do something useful with this
//bool DemoTfListener::serviceCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
//    ROS_INFO("service callback activated");
//    response.resp = true; // boring, but valid response info
//    return true;
//}


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "demoTfListener"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type DemoTfListener");
    DemoTfListener demoTfListener(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate, e.g. 50Hz
   
    ROS_INFO:("starting main loop");
    while (ros::ok()) {
        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

