/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef _GRASP_EXECUTION_EXCEPTIONS_H_
#define _GRASP_EXECUTION_EXCEPTIONS_H_

#include <stdexcept>

namespace object_manipulator {

//! General base class for all exceptions originating in the grasping pipeline
class GraspException : public std::runtime_error
{ 
 public:
 GraspException(const std::string error) : std::runtime_error("grasp execution:"+error) {};
};

//! Thrown if the user has requested an interruption
class InterruptRequestedException : public GraspException
{ 
public:
  InterruptRequestedException() : GraspException("interrupt requested") {};
};

//! Thrown if the current robot state prevents any grasp execution
class IncompatibleRobotStateException : public GraspException
{
 public:
 IncompatibleRobotStateException(const std::string error) : GraspException("robot_state:"+error){};
};

//! Thrown if move arm can not plan out of the current arm state
/*! No other grasps should be attempted, since move arm can not get the arm out
  of its current state.
*/
class MoveArmStuckException : public IncompatibleRobotStateException
{
 public:
 MoveArmStuckException() : IncompatibleRobotStateException("move arm stuck"){};
};

//! Thrown when the grasping pipeline has failed to communicate with or issue commands to the robot
/*! These are not algorithmic failures; they indicate that some form of communication or command
  with the robot, which is always expected to work, has actually failed. They generally mean that
  the execution of the grasp has to be stopped.
 */
class MechanismException : public GraspException
{
 public:
 MechanismException(const std::string error) : GraspException("mechanism:"+error) {};
};

//! Thrown when a service or action server was not found
/*! These generally mean that either a needed launch file was not launched, a node has crashed
  or some topics ar action names are mismatched.
*/
class ServiceNotFoundException : public MechanismException
{
 public:
 ServiceNotFoundException(const std::string service_name) : 
  MechanismException("service or action not found:"+service_name) {};
};

//! Thrown when the grasping pipeline has failed to communicate with the collision map
/*! These are not algorithmic failures; they indicate that some form of communication with the robot, 
  which is always expected to work, has actually failed. They generally mean that the execution of the 
  grasp has to be stopped.
 */
class CollisionMapException : public GraspException
{
 public:
 CollisionMapException(const std::string error) : GraspException("collision map:"+error) {};
};

//! Thrown when a needed parameter is not found on the parameter server
class MissingParamException : public GraspException
{
 public:
  MissingParamException(const std::string name) : GraspException("missing parameter:"+name) {};
};

//! Thrown when a needed parameter on the parameter server is found but does not have the right type
class BadParamException : public GraspException
{
 public:
  BadParamException(const std::string name) : GraspException("bad parameter:"+name) {};
};


} //namespace object_manipulator

#endif
