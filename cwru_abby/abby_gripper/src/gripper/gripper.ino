
/* 
 * rosserial Gripper Server
 * 
 * This simple server controls a 2-position (open/closed) parallel plate gripper.
 */

#include <ros.h>
#include <ros/time.h>
#include <abby_gripper/gripper.h>

ros::NodeHandle  nh;
const int gripper_pin = 0;

void callback(abby_gripper::gripperRequest &req, abby_gripper::gripperResponse &resp)
{
  // Set the gripper output to high for open and low for closed
  if(req.command == abby_gripper::gripperRequest::OPEN){
    //Open the gripper
    digitalWrite(gripper_pin, HIGH);
  }
  else if(req.command == abby_gripper::gripperRequest::CLOSE){
    //Close the gripper
    digitalWrite(gripper_pin, LOW);
  }
  if(digitalRead(gripper_pin) == HIGH)
      resp.position = abby_gripper::gripperResponse::OPEN;
  else
      resp.position = abby_gripper::gripperResponse::CLOSED;
  resp.success = abby_gripper::gripperResponse::SUCCESS;
}

ros::ServiceServer<abby_gripper::gripperRequest, abby_gripper::gripperResponse> server = ros::ServiceServer("gripper", callback);

void setup()
{
  pinMode(gripper_pin,OUTPUT);
  nh.initNode();
  nh.advertiseService(server);
}

void loop()
{
  nh.spinOnce();
}

