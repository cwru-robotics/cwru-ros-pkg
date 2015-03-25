//test_traj_point_sender.cpp:
//wsn, March 2015
//a simple node to illustrate how to publish a trajectory to topic "joint_path_command"
// intended to work with example_robot_interface node, 
// but should work as well with real ROS-Industrial "motion_download_interface" node--
// BUT BE CAREFUL THAT THE TRAJECTORY SENT IS SAFE

// this version:  hard-code poses of interest for testing

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
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");    
    ros::Rate sleep_timer(10.0); // update rate
    
    // may need this to get publisher advertisement recognized
   for (int i=0;i<10;i++) {
            ros::spinOnce();
            sleep_timer.sleep();
            }
    
    /*
    ROS_INFO("sending empty trajectories");
    for (int i=0;i<5;i++)
    {
        pub.publish(new_trajectory);
            ros::spinOnce();
            sleep_timer.sleep();
    }
    /**/
    // build an example trajectory:
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear();    
    
    
    //allocate memory for 6 joint position commands
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point1.positions.push_back(0.0); // stuff in position commands for 6 joints
        //should also fill in trajectory_point.time_from_start      
    }
    
    //manual entry for a test point of interest
    trajectory_point1.positions[0] = 0.1; //amp*sin(theta); //sign*amp;
    trajectory_point1.positions[1] = 0.2; //amp*sin(theta); 
    trajectory_point1.positions[2] = 0.3; //amp*sin(theta); 
    trajectory_point1.positions[3] = 0.4; //amp*sin(theta); 
    trajectory_point1.positions[4] = 0.5; //amp*sin(theta); 
    trajectory_point1.positions[5] = 0.6; //amp*sin(theta);  
    
    // try sending a sinusoid or square wave
    double sign = -1.0; // to make square wave
    double amp = 0.5; // amplitude of square wave or sin wave
    double dtheta = 0.1; // step size of sin wave
    double theta = 0.0; // should start from a reasonable angle...
    double dt = 0.2;
    double t = 0.0;
    ros::Duration t_from_start(0); //initialize duration to 0
    trajectory_point1.time_from_start =    t_from_start; 
        new_trajectory.points.clear();
        new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector
        //new_trajectory.points.t
        for (int i=0;i<100;i++) {
            //sign*= -1.0;
            theta+=dtheta;
            trajectory_point1.positions[0] = amp*sin(theta); //sign*amp;
            trajectory_point1.positions[1] = amp*sin(theta); 
            trajectory_point1.positions[2] = amp*sin(theta); 
            trajectory_point1.positions[3] = amp*sin(theta); 
            trajectory_point1.positions[4] = amp*sin(theta); 
            trajectory_point1.positions[5] = amp*sin(theta);  
            t += dt;
            
            trajectory_point1.time_from_start =    ros::Duration(t);
            ROS_INFO("using time-from-start = %f",trajectory_point1.time_from_start.toSec());
            new_trajectory.points.push_back(trajectory_point1);
        }
        
        //new_trajectory.points.push_back(trajectory_point2); // append another point
        
        new_trajectory.header.stamp = ros::Time::now();
     int npts = new_trajectory.points.size(); 
     int njnts = new_trajectory.points[0].positions.size();
    ROS_INFO("sending a trajectory with %d poses, each with %d joints ",npts,njnts);

     // stick around for a while; if die too early, perhaps message does not get sent (??)
            pub.publish(new_trajectory);
            for (int i=0;i<10;i++) {
            ros::spinOnce();
            sleep_timer.sleep();
            }
    return 0;
} 

