#include <ros/ros.h>

#ifndef CWRU_WSN_STEERING_STEERING_BASE_H_
#define CWRU_WSN_STEERING_STEERING_BASE_H_

namespace cwru_wsn_steering
{
	class SteeringBase
	{
		public:
			virtual void initialize(ros::NodeHandle nh_) = 0;
			/*
			 * x,y in meters in ROS frame
			 * psi in rads in ROS frame, 0 points to true north
			 * v in meters/sec forwards velocity
			 * omega in rads/sec
			 */
			virtual void computeVelocities(double x_PSO, double y_PSO, double psi_PSO, double x_des, double y_des, double v_des, double psi_des, double rho_des, double &v, double &omega) = 0;
			virtual ~SteeringBase(){}

		protected:
			SteeringBase(){}
	};
};
#endif
