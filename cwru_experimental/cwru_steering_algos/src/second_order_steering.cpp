#include <ros/ros.h>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <cwru_wsn_steering/steering_base.h>
#include <cwru_steering_algos/second_order_steering.h>

PLUGINLIB_DECLARE_CLASS(cwru_steering_algos, SecondOrderSteering, second_order_steering::SecondOrderSteering, cwru_wsn_steering::SteeringBase)

	namespace second_order_steering {
		SecondOrderSteering::SecondOrderSteering() {}

		void SecondOrderSteering::initialize(ros::NodeHandle nh_) {
			double convergence_rate; //convergence rate in meters
			nh_.param("convergence_rate", convergence_rate, 2.0); 
			nh_.param("k_v", k_v, 1.0);

			//Computing gains based on convergence_rate parameter
			k_d = 1.0/pow(convergence_rate, 2.0);
			k_psi = 2.0/convergence_rate;
			ROS_INFO("Second Order Steering initialized");
		}

		void SecondOrderSteering::computeVelocities(double x_PSO, double y_PSO, double psi_PSO, double x_des, double y_des, double v_des, double psi_des, double rho_des, double &v, double &omega) {
			const double pi = 3.1415926;
			double tanVec[2],nVec[2],dx_vec[2],d;
			double deltaPsi;

			tanVec[0]= cos(psi_des); //-sin(psi_des); // vector tangent to desired lineseg
			tanVec[1]= sin(psi_des); //cos(psi_des); 

			nVec[0]= -tanVec[1];  // normal vector of desired (directed) lineseg--points "left" of heading
			nVec[1]=  tanVec[0];
			dx_vec[0] = x_des-x_PSO;
			dx_vec[1] = y_des-y_PSO; //position error
			double Lfollow = tanVec[0]*dx_vec[0]+tanVec[1]*dx_vec[1];

			//Check if the desired speed is 0
			//If so we will just set it there and be done with velocity computation
			if (fabs(v_des) < 1e-7) {
				ROS_DEBUG("v_des was %f, abs was %f, so setting it to 0", v_des, fabs(v_des));
				v = 0.0;
			} else {	
				v = v_des + k_v * Lfollow;
			}
			ROS_DEBUG("V_des was %f, v was %f, k_v was %f, Lfollow was %f", v_des, v, k_v, Lfollow);
			// d = -n'*dx_vec;
			d = -nVec[0]*dx_vec[0]-nVec[1]*dx_vec[1];
			deltaPsi = psi_PSO-psi_des;
			//std::cout << "psi_PSO " << psi_PSO << " " << "psi_des " << psi_des << std::endl;
			while (deltaPsi>pi)
				deltaPsi-=2*pi;
			while (deltaPsi< -pi)
				deltaPsi+=2*pi;
			double rho_cmd = -k_d*d -k_psi*deltaPsi + rho_des;
			//std::cout << "dPsi = " << deltaPsi  << " " << "d = " << d << std::endl;
			omega = v*rho_cmd;
			//std::cout << "omega = " << omega << std::endl;
		}
	};
