/** interactive_path_maker.h header file **/
#ifndef INTERACTIVE_PATH_MAKER_H_
#define INTERACTIVE_PATH_MAKER_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <cwru_srv/simple_bool_service_message.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>

class InteractivePathMaker
{
public:
    InteractivePathMaker(); //constructor
    InteractivePathMaker(ros::NodeHandle* nodehandle);
    ros::Publisher  nav_path_pub_;
private:
    // put member data here
    ros::NodeHandle nh_; 
    ros::Subscriber nav_goal_callback_sub_;
    ros::Subscriber minimal_subscriber_;

    ros::ServiceServer nav_trigger_service_;
    
    // member methods as well:
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    
    void navGoalCallback(const geometry_msgs::PoseStamped::ConstPtr navPoseMsg);
    //void myCallback(const std_msgs::Float32& message_holder);
    //void myCallback(const std_msgs::Float32ConstPtr& message_holder);
    void myCallback(const std_msgs::Float32ConstPtr& message_holder);
    bool triggerCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
    //void gravityTorquesCB(const hku_msgs::vectorOfDoublesWHeaderConstPtr& message_holder);
};

#endif
