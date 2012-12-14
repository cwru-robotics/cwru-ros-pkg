
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
const int gripper_pin = 13;
boolean position = LOW;

#define CLOSED_POSITION -0.03
#define OPEN_POSITION -0.039

sensor_msgs::JointState joint_msg;
ros::Publisher joint_pub("joint_states", &joint_msg);
float joint_velocities[2] = {0.0, 0.0};
float joint_efforts[2] = {0.0, 0.0};
float joint_positions[2] = {OPEN_POSITION, OPEN_POSITION};
char * joint_names[14] = {"gripper_jaw_1", "gripper_jaw_2"};

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
  
  joint_msg.name_length = joint_msg.velocity_length = joint_msg.effort_length = joint_msg.position_length = 2;
  joint_msg.name = joint_names;
  joint_msg.velocity = joint_velocities;
  joint_msg.effort = joint_efforts;
  joint_msg.position = joint_positions;
  
  nh.initNode();
  nh.advertiseService(server);
  nh.advertise(joint_pub);
}

void loop()
{
  if(position){
      joint_positions[0] = OPEN_POSITION;
      joint_positions[1] = OPEN_POSITION;
  }
  else{
      joint_positions[0] = CLOSED_POSITION;
      joint_positions[1] = CLOSED_POSITION;
  }
  joint_msg.header.stamp = nh.now();
  joint_pub.publish(&joint_msg);
  nh.spinOnce();
  delay(50);
}
