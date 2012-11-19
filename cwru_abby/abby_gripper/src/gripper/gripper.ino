
/* 
 * rosserial Gripper Server
 * 
 * This simple server controls a 2-position (open/closed) parallel plate gripper.
 */

#include <ros.h>
#include <ros/time.h>
#include <abby_gripper/gripper.h>

ros::NodeHandle  nh;
const int gripper_pin = 13;
boolean position = LOW;

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
  nh.initNode();
  nh.advertiseService(server);
}

void loop()
{
  nh.spinOnce();
}
