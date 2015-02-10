//simple interactive motion commander v2:
// this version (v2) also responds to 2D Nav Goal inputs from Rviz;
// such inputs trigger automatically--do not require external trigger
// ALSO, with rviz in "map" as fixed frame, this version uses tf
// to convert goal coords from map frame to odom frame

// receive goal coords from interactive marker
// receive "execute" signals via ROS service client
// execute motion as: 1) spin towards goal; 2) move towards goal x,y; 3: spin to align w/ target orientation

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <cwru_srv/simple_bool_service_message.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
 #include <tf/transform_listener.h>
#include <math.h>

// mnemonics:
const int STAND_STILL = 0;
const int INIT_SPIN = 1;
const int FWD_MOVE = 2;
const int FINAL_SPIN = 3;
const double OMEGA = 0.1;
const double VEL = 0.1;

//globals:
//geometry_msgs::Point marker_point_;
double marker_x_ = 0.0;
double marker_y_ = 0.0;
double marker_phi_ = 0.0;
//geometry_msgs::Point vertex_start;
double target_phi_ = 0.0;
double target_x_ = 0.0;
double target_y_ = 0.0;

// globals for communication w/ callbacks:
double odom_vel_ = 0.0; // measured/published system speed
double odom_omega_ = 0.0; // measured/published system yaw rate (spin)
double odom_x_ = 0.0;
double odom_y_ = 0.0;
double odom_phi_ = 0.0;

double del_phi_init_ = 0.0;
double del_phi_final_ = 0.0;
double phi_start_to_goal_ = 0.0;
double del_dist_ = 0.0;


tf::TransformListener* tfListener_;



bool new_goal_ = false;

//function to strip off sign of x

double sgn(double x) {
    if (x > 0.0) return 1.0;
    else if (x < 0.0) return -1.0;
    else return 0.0;

}

void init_move(void) {
    new_goal_ = true; // we received a request, so interpret this as a trigger

    //compute the initial heading change, distance to travel, and terminal heading change
    target_x_ = marker_x_;
    target_y_ = marker_y_;
    target_phi_ = marker_phi_;
    ROS_INFO(" desired x,y,phi = %f, %f, %f", target_x_, target_y_, target_phi_);
    double dx = marker_x_ - odom_x_;
    double dy = marker_y_ - odom_y_;
    del_dist_ = sqrt(dx * dx + dy * dy);
    if (del_dist_ > 0.1) {
        phi_start_to_goal_ = atan2(dy, dx);
    } else { // for tiny move distance, skip translation and only do rotation
        phi_start_to_goal_ = target_phi_;
        del_dist_ = 0.0;
    }
    del_phi_init_ = phi_start_to_goal_ - odom_phi_;
    //choose smallest periodic solution to desired heading change;
    if (del_phi_init_ > M_PI)
        del_phi_init_ -= 2 * M_PI;
    if (del_phi_init_<-M_PI)
        del_phi_init_ += 2 * M_PI;

    del_phi_final_ = target_phi_ - phi_start_to_goal_;
    if (del_phi_final_ > M_PI)
        del_phi_final_ -= 2 * M_PI;
    if (del_phi_final_<-M_PI)
        del_phi_final_ += 2 * M_PI;

    ROS_INFO("dphi init, phi to goal, dphi final, dist to goal = %f %f %f %f", del_phi_init_, phi_start_to_goal_, del_phi_final_, del_dist_);

    
}

void processFeedbackFinalPose(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << "marker is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    marker_x_ = feedback->pose.position.x;
    marker_y_ = feedback->pose.position.y;

    double quat_z = feedback->pose.orientation.z;
    double quat_w = feedback->pose.orientation.w;
    marker_phi_ = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion

    ROS_INFO_STREAM("heading:   " << marker_phi_);
}

void navGoalCallback(
        const geometry_msgs::PoseStamped::ConstPtr navPoseMsg) {
    //tf::StampedTransform mapToOdom;
    //tf::Stamped<tf::Pose> navGoalInMapFrame; 
    //tf::Stamped<tf::Pose> navGoalInOdomFrame;    
    //tf::poseStampedMsgToTF(*navPoseMsg, navGoalInMapFrame);
    geometry_msgs::PoseStamped navGoalInOdomFrame;
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    double map_goal_x = navPoseMsg->pose.position.x;
    double map_goal_y = navPoseMsg->pose.position.y;
    //double map_goal_phi;
    //marker_x_ = feedback->pose.position.x;
    //marker_y_ = feedback->pose.position.y;

    //double quat_z = navPoseMsg->pose.orientation.z;
    //double quat_w = navPoseMsg->pose.orientation.w;
    //map_goal_phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    ROS_INFO("new nav goal in map frame: x,y = %f, %f",map_goal_x,map_goal_y);
    //ROS_INFO_STREAM("heading:   " << map_goal_phi);
    //tfListener_->lookupTransform("odom", "map", ros::Time(0), mapToOdom);
    //navGoalInOdomFrame = mapToOdom*navGoalInMapFrame;
    
    tfListener_->transformPose("odom",*navPoseMsg,navGoalInOdomFrame);
    
    
    // tf::Transform handle_pose_in_torso_frame = door_to_torso * handle_pose_in_door_frame; 
    //navGoalInOdomFrame = mapToOdom*navGoalInMapFrame;
    marker_x_ = navGoalInOdomFrame.pose.position.x;
    marker_y_ = navGoalInOdomFrame.pose.position.y;

    double quat_z = navGoalInOdomFrame.pose.orientation.z;
    double quat_w = navGoalInOdomFrame.pose.orientation.w;
    marker_phi_ = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    ROS_INFO("nav goal in  odom frame: x,y = %f, %f",marker_x_,marker_y_);
    ROS_INFO("heading:   %f",marker_phi_);   
    init_move();
    
}

void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    // copy some of the components of the received message into global vars, for use by "main()"
    // we care about speed and spin, as well as position estimates x,y and heading
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    // see notes above for conversion for simple planar motion
    double quat_z = odom_rcvd.pose.pose.orientation.z;
    double quat_w = odom_rcvd.pose.pose.orientation.w;
    odom_phi_ = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion

}


bool triggerCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("path goal trigger callback activated");
    response.resp = true; // not really useful in this case--and not necessary
    init_move();
    return true;
}

int main(int argc, char **argv) {
    double dt=0.01;
    ros::init(argc, argv, "interactive_robot_commander"); // name of this node 
    ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
    //stdr "robot0" is expecting to receive commands on topic: /robot0/cmd_vel
    // commands are of type geometry_msgs/Twist, but they use only velocity in x dir and
    //  yaw rate in z-dir; other 4 fields will be ignored
    ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // change topic to command abby...
    //ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("abby/cmd_vel",1);

    ros::Subscriber subFinal = nh.subscribe("/path_end/feedback", 1, processFeedbackFinalPose);
    ros::Subscriber subNavGoal = nh.subscribe("/move_base_simple/goal", 1, navGoalCallback);    
    
    ros::Subscriber subOdom = nh.subscribe("/odom", 1, odomCallback);
    ros::ServiceServer service = nh.advertiseService("trigger_path_goal", triggerCallback);

    ros::Rate sleep_timer(1/dt); //let's make a 100Hz timer
    
    odom_phi_ = -1000; // impossible value; fix this with odom call    
    while (odom_phi_ < -100) {
        ROS_INFO("waiting for odom");
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
    // now have a valid odom value, so init marker to these values
    ROS_INFO("got valid odom value");
    marker_x_ = odom_x_;
    marker_y_ = odom_y_;   
    marker_phi_ = odom_phi_;
    
    //tf::TransformListener tfListener;
    tfListener_ = new tf::TransformListener; 
    tf::StampedTransform mapToOdom;    
    bool tferr=true;
    ROS_INFO("waiting for tf...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener_->lookupTransform("odom", "map", ros::Time(0), mapToOdom);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    //tfListener.transform 
    
    //create a variable of type "Twist", as defined in: /opt/ros/hydro/share/geometry_msgs
    // any message published on a ROS topic must have a pre-defined format, so subscribers know how to
    // interpret the serialized data transmission
    geometry_msgs::Twist twist_cmd;
    // look at the components of a message of type geometry_msgs::Twist by typing:
    // rosmsg show geometry_msgs/Twist
    // It has 6 fields.  Let's fill them all in with some data:
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    int phase = 0;
    double phase1_spin_to_go = 0.0;
    double dist_to_go = 0.0;
    double phase3_spin_to_go = 0.0;
    double phase1_omega = 0.0;
    double phase3_omega = 0.0;
    double phase1_spin_sign = 0.0;
    double phase3_spin_sign = 0.0;
    double fwd_speed = 0.0;
    double phi_tol = OMEGA*dt*1.5;
    double fwd_tol = VEL*dt*1.5;
    while (ros::ok()) {
        // if we have a new goal (from trigger), initialize values for a new 3-part move
        if (new_goal_) {
            ROS_INFO("main: new goal");
            new_goal_ = false; //reset the trigger
            phase = INIT_SPIN; //restart the spin-move-spin sequence
            //initialize distances to go:
            phase1_spin_to_go = del_phi_init_;
            phase1_spin_sign = sgn(phase1_spin_to_go);
            ROS_INFO("phi to go: %f; sgn: %f",phase1_spin_to_go,phase1_spin_sign);
            dist_to_go = del_dist_;
            phase3_spin_to_go = del_phi_final_;
            phase3_spin_sign = sgn(phase3_spin_to_go);
            // set speeds for 3 phases:
            phase1_omega = phase1_spin_sign*OMEGA;
            phase3_omega = phase3_spin_sign*OMEGA;
            fwd_speed = VEL;
        }

        // move incrementally through 3 phases
        switch (phase) {
            case INIT_SPIN:
                ROS_INFO("INIT_SPIN; omega = %f",phase1_omega);
                    twist_cmd.linear.x = 0.0;
                    twist_cmd.angular.z = phase1_omega;
                    phase1_spin_to_go-= phase1_omega*dt;
                    if (phase1_spin_to_go*phase1_spin_sign<phi_tol) { //test for done with spin 
                        phase=FWD_MOVE;
                    }
                break;
            case FWD_MOVE:
                ROS_INFO("FWD_MOVE");
                    twist_cmd.linear.x = fwd_speed;
                    twist_cmd.angular.z = 0.0;
                    dist_to_go-= fwd_speed*dt;
                    if (dist_to_go<fwd_tol) { //test for done with fwd motion 
                        phase=FINAL_SPIN;
                    }
                break;
            case FINAL_SPIN:
                ROS_INFO("FINAL_SPIN");
                    twist_cmd.linear.x = 0.0;
                    twist_cmd.angular.z = phase3_omega;
                    phase3_spin_to_go-= phase3_omega*dt;
                    if (phase3_spin_to_go*phase3_spin_sign<phi_tol) { //test for done with spin 
                        phase=STAND_STILL;
                        ROS_INFO("STAND_STILL");
                    }
                break;

            case STAND_STILL:
            default:
                //ROS_INFO("STAND_STILL");
                twist_cmd.linear.x = 0.0;
                twist_cmd.linear.y = 0.0;
                twist_cmd.linear.z = 0.0;
                twist_cmd.angular.x = 0.0;
                twist_cmd.angular.y = 0.0;
                twist_cmd.angular.z = 0.0;
        }

        cmd_publisher.publish(twist_cmd); // 
        sleep_timer.sleep(); // sleep for (remainder of) 10ms
        ros::spinOnce();
    }


    return 0;
} 