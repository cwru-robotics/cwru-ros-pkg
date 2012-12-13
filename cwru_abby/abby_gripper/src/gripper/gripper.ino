
/* 
 * rosserial Gripper Server
 * 
 * This simple server controls a 2-position (open/closed) parallel plate gripper.
 */

#include <ros.h>
#include <ros/time.h>
#include <abby_gripper/gripper.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;
sensor_msgs::JointState joint_msg;
ros::Publisher joint_pub("joint_states", &joint_msg);
const int gripper_pin = 13;
boolean position = LOW;

#define CLOSED_POSITION -0.023
#define OPEN_POSITION -0.031

void callback(const abby_gripper::gripperRequest &req, abby_gripper::gripperResponse &resp){
  // Set the gripper output to high for open and low for closed
  if(req.command == abby_gripper::gripperRequest::OPEN){
    //Open the gripper
    position = HIGH;
    digitalWrite(gripper_pin, position);
  }
  else if(req.command == abby_gripper::gripperRequest::CLOSE){
    //Close the gripper
    position = LOW;
    digitalWrite(gripper_pin, position);
  }
  if(position)
      resp.position = abby_gripper::gripperResponse::OPEN;
  else
      resp.position = abby_gripper::gripperResponse::CLOSED;
  resp.success = abby_gripper::gripperResponse::SUCCESS;
}

ros::ServiceServer<abby_gripper::gripperRequest, abby_gripper::gripperResponse> server("abby_gripper/gripper", &callback);

void setup(){
  pinMode(gripper_pin,OUTPUT);
  
  joint_msg.name[0] = "gripper_jaw_1";
  joint_msg.name[1] = "gripper_jaw_2";
  joint_msg.velocity[0] = 0.0;
  joint_msg.velocity[1] = 0.0;
  joint_msg.effort[0] = 0.0;
  joint_msg.effort[1] = 0.0;
  
  nh.initNode();
  nh.advertiseService(server);
  nh.advertise(joint_pub);
}

void loop()
{
  nh.spinOnce();
  if(position){
      joint_msg.position[0] = OPEN_POSITION;
      joint_msg.position[1] = OPEN_POSITION;
  }
  else{
      joint_msg.position[0] = CLOSED_POSITION;
      joint_msg.position[1] = CLOSED_POSITION;
  }
  joint_pub.publish(&joint_msg);
  delay(50);
}
