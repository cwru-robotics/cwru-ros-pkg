//simple interactive motion commander:
// receive goal coords from interactive marker
// receive "execute" signals via ROS service client
// execute motion as: 1) spin towards goal; 2) move towards goal x,y; 3: spin to align w/ target orientation
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <cwru_srv/simple_bool_service_message.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>

//globals:
geometry_msgs::Point vertex_final;
geometry_msgs::Point vertex_start;
double heading_final;
double heading_start;

bool new_goal=false;


void processFeedbackFinalPose(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    vertex_final.x = feedback->pose.position.x;
    vertex_final.y = feedback->pose.position.y;
    vertex_final.z = 0.0;    
    ROS_INFO_STREAM("orientation: z = "<<feedback->pose.orientation.z<<"; w = "<<feedback->pose.orientation.w);
}

bool triggerCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("path goal trigger callback activated");
    new_goal=true; // we received a request, so interpret this as a trigger
    response.resp = true; // not really useful in this case--and not necessary
  return true;
}

int main(int argc, char **argv)
{
ros::init(argc,argv,"interactive_robot_commander"); // name of this node 
ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
//stdr "robot0" is expecting to receive commands on topic: /robot0/cmd_vel
// commands are of type geometry_msgs/Twist, but they use only velocity in x dir and
//  yaw rate in z-dir; other 4 fields will be ignored
ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1);
// change topic to command abby...
//ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("abby/cmd_vel",1);

ros::Subscriber subFinal = nh.subscribe("/path_end/feedback", 1, processFeedbackFinalPose);

ros::ServiceServer service = nh.advertiseService("trigger_path_goal", triggerCallback);
  
ros::Rate sleep_timer(100); //let's make a 100Hz timer

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

ros::spin(); // DEBUG

twist_cmd.linear.x = 0.4;


// timer test...print out a message every 1 second
ROS_INFO("count-down");
for (int j=3;j>0;j--) {
    ROS_INFO("%d",j);
    for (int i = 0; i<100;i++)
        sleep_timer.sleep();
}

int niters = 1200; //1000 iters at 100Hz is 10 seconds;
//iteration counter; at 10ms/iteration, and 0.2m/sec, expect 2mm/iter
// should move by 2m over 10 sec
for (int i=0;i<niters;i++) {
    cmd_publisher.publish(twist_cmd); // really, should only need to publish this once, but no hard done
    sleep_timer.sleep(); // sleep for (remainder of) 10m
}
twist_cmd.linear.x = 0.0;
twist_cmd.angular.z = -0.314;
niters=500; // 5 sec
ROS_INFO("Time to rotate negative");
for (int i=0;i<niters;i++) {
    cmd_publisher.publish(twist_cmd); // really, should only need to publish this once, but no hard done
    sleep_timer.sleep(); // sleep for (remainder of) 10m
}
ROS_INFO("my work here is done");
//while (ros::ok()) 
{
twist_cmd.linear.x = 0.0;
twist_cmd.angular.z = 0;
cmd_publisher.publish(twist_cmd); // and halt
}


return 0;
} 