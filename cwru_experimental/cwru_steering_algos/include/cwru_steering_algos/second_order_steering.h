#include <ros/ros.h>
#include <cwru_wsn_steering/steering_base.h>

#ifndef CWRU_WSN_STEERING_SECOND_ORDER_STEERING_H_
#define CWRU_WSN_STEERING_SECOND_ORDER_STEERING_H_

namespace second_order_steering
{
	class SecondOrderSteering : public cwru_wsn_steering::SteeringBase
	{
		public:
			SecondOrderSteering();
			void initialize(ros::NodeHandle nh_);
			void computeVelocities(double x_PSO, double y_PSO, double psi_PSO, double x_des, double y_des, double v_des, double psi_des, double rho_des, double &v, double &omega);
		private:
			double k_psi;
			double k_v;
			double k_d;

	};
};
#endif
