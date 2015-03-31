//test_abby_sender2.cpp:
// use this variation to test on actual robot, using topic "/command"
// ALSO subscribe to joint states
//wsn, March 2015
//a simple node to illustrate how to publish a trajectory to topic "joint_path_command"
// intended to work with example_robot_interface node, 
// but should work as well with real ROS-Industrial "motion_download_interface" node--
// BUT BE CAREFUL THAT THE TRAJECTORY SENT IS SAFE

// try a square wave...try w/ update rate = 1Hz

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
#include <sensor_msgs/JointState.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;

using namespace std;

Vectorq6x1 g_q_state;
bool g_trigger=false;

void jointStateCB(
const sensor_msgs::JointStatePtr &js_msg) {
    
    for (int i=0;i<6;i++) {
        g_q_state[i] = js_msg->position[i];
    }
    //cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void stuff_trajectory( Vectorq6x1 qvec, trajectory_msgs::JointTrajectory &new_trajectory) {
    
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
       
    new_trajectory.points.clear();
    new_trajectory.header.stamp = ros::Time::now();  
    
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear(); 
    //fill in the points of the trajectory: initially, all home angles
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point1.positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
        trajectory_point2.positions.push_back(0.0); // stuff in position commands for 6 joints        
    }
    trajectory_point1.time_from_start =    ros::Duration(0);          // fill in trajectory_point.time_from_start
    trajectory_point2.time_from_start =    ros::Duration(2.0);        // take 2 seconds to move to here

    // start from current pose!
    new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector     
    
    // fill in the target pose: really should fill in a sequence of poses leading to this goal
    // ABB IRC5 controller will interpolate to produce a smooth motion nonetheless (but simulator will not)
    for (int ijnt=0;ijnt<6;ijnt++) {
            trajectory_point2.positions[ijnt] = qvec[ijnt]; // target pose from qvec argument
    }  

    new_trajectory.points.push_back(trajectory_point2); // append this point to trajectory
}


bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
 
    g_trigger=true; //inform "main" that we have a new goal!
    return true;
}


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "test_traj_sender"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);  
    ros::Subscriber sub_js = nh.subscribe("/joint_states",1,jointStateCB);
    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);  
 
    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");   
    
    Vectorq6x1 qvec;       
    for (int i=0;i<6;i++) qvec[i]=0.0; // home angles, by default
  
    ros::Rate sleep_timer(10.0); // update rate
    
    // make sure receive valid joint states from robot:
    g_q_state[0] = 1000.0; // impossible value
    while (fabs(g_q_state[0])>10.0) {

            ros::spinOnce();
            ros::Duration(1.0).sleep();
            ROS_INFO("waiting for joint-state callback...");
    }
    ROS_INFO("ready to roll");
    ROS_INFO("to trigger a move:  rosservice call move_trigger 1, or enter 7 for joint number ");
    int jnum;
    double theta;
      while(ros::ok()) {
            ros::spinOnce();
            cout<<"enter joint num, 0 through 5; <0 to quit; 7 to execute: ";
            cin>>jnum;
            if (jnum<0) return 0;
            

            if (jnum>=0 && jnum <= 5) {
                cout<<"enter desired angle for joint "<<jnum<<": ";
                cin>>theta;
                qvec[jnum]=theta;
                cout<<"goal angles: "<<qvec.transpose()<<endl;
            }
            if (jnum==7) g_trigger=true;
            if (g_trigger) {
                ROS_INFO("got a trigger: transferring trajectory");
                g_trigger=false; // reset the trigger  

                 stuff_trajectory(qvec,new_trajectory);
                int npts = new_trajectory.points.size(); 
                int njnts = new_trajectory.points[0].positions.size();
                ROS_INFO("sending a trajectory with %d poses, each with %d joints ",npts,njnts); 
                 pub.publish(new_trajectory);
            }
            sleep_timer.sleep();
      }
           
    return 0;
} 

