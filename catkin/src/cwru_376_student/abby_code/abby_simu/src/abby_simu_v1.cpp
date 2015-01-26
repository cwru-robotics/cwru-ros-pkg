// primitive simulator--base motion from no-slip wheel dynamics and wheel velocity commands
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h> //message to send pose commands to gazebo
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream> //for debug I/O
//#include <Eigen/Dense>  // not using "Eigen" package yet
//#include <Eigen/Core>

using namespace std; // for cin and cout

//declaring this here makes it global; 
gazebo_msgs::ModelState abbyState; // instance of message type to command abby pose
geometry_msgs::Twist g_cmd_vel; //global var to store incoming velocity commands

const double DT= 0.001; // intended simu time step

// notes on quaternions:
/*
From:
http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/

qx = ax * sin(angle/2)
qy = ay * sin(angle/2)
qz = az * sin(angle/2)
qw = cos(angle/2)


so, quaternion in 2-D plane (x,y,theta):
ax=0, ay=0, az = 1.0

qx = 0;
qy = 0;
qz = sin(angle/2)
qw = cos(angle/2)

therefore, theta = 2*atan2(qz,qw)
*/

        
void myCallback(const geometry_msgs::Twist& cmd_vel) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "topic1"
    // Since this function is prompted by a message event, it does not consume CPU time polling for new data
    // the ROS_INFO() function is like a printf() function, except
    // it publishes its output to the default rosout topic, which prevents
    // slowing down this function for display calls, and it makes the
    // data available for viewing and logging purposes
    //ROS_INFO("received value is: vx= %f  omega_z = %f", cmd_vel.linear.x,cmd_vel.angular.z);
    g_cmd_vel = cmd_vel; // copy the data to global variable g_cmd_vel for use by main()
    //really could do something interesting here with the received data...but all we do is print it
}


void init_state(); //a utility function to initialize Abby state


int main(int argc, char** argv) {

    // ROS set-ups:
    ros::init(argc, argv, "abby_simu");

    ros::NodeHandle nh;
    //here is the publisher to send model state to gazebo
    ros::Publisher pub_model_state= nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1, true);
    ros::Subscriber subscribe_cmd_vel = nh.subscribe("/abby/cmd_vel", 1, myCallback);

    ros::Rate looprate(1/DT); //timer to sleep for 1 msec
 
    //ROS_INFO("setting up subscriber to atlas state");
    //ros::Subscriber sub = nh.subscribe("/atlas/atlas_state", 1, getJointStatesCB);

    // initial state: These get updated by Euler 1-step integration
    double theta_z = 0.0;
    double pose_x = 0.0;
    double pose_y = 0.0;   
    
    // speed and spin: these get updated by the callback via cmd_vel topic
    double omega_z = 0.0; // angular velocity of abby
    double speed=0.0;
    
    //these are increments, each time step:
    double dx = 0; // inc displacement in world x coords
    double dy = 0; // inc displacement in world y coords
    double dtheta_z = 0.0; // inc rotation
    double ds = 0.0; //inc forward motion
    double dt = 0.0; // actual time step
    ros::Time t_last_iter; // use this to ask the system to compute the true time step
        
    t_last_iter = ros::Time::now();  // system clock time   
    
    init_state(); // populate model state message with initial data;
    
  
    ROS_INFO("starting simu loop...");
    // main loop
    int iter=0;
    while (ros::ok()) {
        dt = (ros::Time::now() - t_last_iter).toSec(); // how long have we been asleep?
        if (dt>0.003)
            dt = 0.003; // watch out for dt too large.
        t_last_iter = ros::Time::now(); // prep for next dt computation

        speed = g_cmd_vel.linear.x;  // these are updated by the cmd_vel callback
        omega_z = g_cmd_vel.angular.z;
        
        ds = speed*dt;  // incremental forward motion 
        dtheta_z = omega_z*dt; //incremental rotation of heading
       
        dx = ds*cos(theta_z);  // displacement in world depends on heading
        dy = ds*sin(theta_z);
        
       // cout<<"dx = "<<dx<<"; dy = "<<dy<<"; dtheta_z = "<<dtheta_z<<endl;

        // accumulate the incremental changes:
        theta_z+=dtheta_z; 
        pose_x += dx; //dx;
        pose_y += dy; //dy;
                //cout<<"pose_x= "<<pose_x<<"; pose_y = "<<pose_y<<"; theta_z = "<<theta_z<<endl;
               //cout<<endl;
        abbyState.pose.position.x=pose_x; //
        abbyState.pose.position.y=pose_y; //        

        abbyState.pose.orientation.x = 0;  // for z axis upright, this is always 0
        abbyState.pose.orientation.y = 0;   // this is also always 0
        //computation of quaternion for pure z rotation is simple...
        abbyState.pose.orientation.z = sin(theta_z/2.0);   // changes with theta_z
        abbyState.pose.orientation.w = cos(theta_z/2.0);   // must balance w^2 + z^2 = 1
        
        // could also update the velocities in abbyState here...not done yet.
    
        // force this pose on Gazebo.  Really, this bypasses the physics engine, so
        // don't expect realistic dynamics--including collisions
        pub_model_state.publish(abbyState);  
        
        //let the callback service its messages to get current wheel speed commands (and later, arm position commands)        
        ros::spinOnce();
        looprate.sleep(); // let's take a quick nap

    }
    return 0;
}

// initialize abby's pose:
void init_state() {
    abbyState.model_name = "abby";
    abbyState.reference_frame = "world";
    abbyState.pose.position.x = 0.0;
    abbyState.pose.position.y = 0.0;    
    abbyState.pose.position.z = 0.0; 
    abbyState.pose.orientation.x = 0.0;      
    abbyState.pose.orientation.y = 0.0;   
    abbyState.pose.orientation.z = 0.0;   
    abbyState.pose.orientation.w = 1.0; 
    abbyState.twist.linear.x=0.0;
    abbyState.twist.linear.y=0.0;
    abbyState.twist.linear.z=0.0;
    abbyState.twist.angular.x=0.0;
    abbyState.twist.angular.y=0.0;
    abbyState.twist.angular.z=0.0;   
    g_cmd_vel.linear.x = 0.0;
    g_cmd_vel.angular.z = 0.0;
 
}
