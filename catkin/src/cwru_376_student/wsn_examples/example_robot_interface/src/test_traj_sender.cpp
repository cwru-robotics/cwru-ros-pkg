//test_traj_sender.cpp:
//wsn, March 2015
//a simple node to illustrate how to publish a trajectory to topic "joint_path_command"
// intended to work with example_robot_interface node, 
// but should work as well with real ROS-Industrial "motion_download_interface" node--
// BUT BE CAREFU THAT THE TRAJECTORY SENT IS SAFE

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "test_traj_sender"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    
    //ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate to send new traj points as commands
    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    
    new_trajectory.points.clear();
    new_trajectory.joint_names.push_back("joint1");
    new_trajectory.joint_names.push_back("joint2");
    new_trajectory.joint_names.push_back("joint3");
    new_trajectory.joint_names.push_back("joint4");
    new_trajectory.joint_names.push_back("joint5");
    new_trajectory.joint_names.push_back("joint6");    
    ros::Rate sleep_timer(1.0); //1Hz update rate
    
    ROS_INFO("sending empty trajectories");
    for (int i=0;i<5;i++)
    {
        pub.publish(new_trajectory);
            ros::spinOnce();
            sleep_timer.sleep();
    }
    
    // build an example trajectory:
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear();    
    
    //specify two points:
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point1.positions.push_back(((double) ijnt)/10.0); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start
        trajectory_point2.positions.push_back(((double) ijnt)/5.0); // stuff in position commands for 6 joints        
    }
    
    
        new_trajectory.points.clear();
        new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector
        new_trajectory.points.push_back(trajectory_point2); // append another point
        
        new_trajectory.header.stamp = ros::Time::now();
     int npts = new_trajectory.points.size(); 
     int njnts = new_trajectory.points[0].positions.size();
    ROS_INFO("sending a trajectory with %d poses, each with %d joints ",npts,njnts);

            pub.publish(new_trajectory);
            ros::spinOnce();
            sleep_timer.sleep();
    return 0;
} 

