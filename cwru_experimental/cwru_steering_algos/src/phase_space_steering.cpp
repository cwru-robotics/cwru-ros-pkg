#include <ros/ros.h>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <cwru_wsn_steering/steering_base.h>
#include <cwru_steering_algos/phase_space_steering.h>

PLUGINLIB_DECLARE_CLASS(cwru_steering_algos, PhaseSpaceSteering, phase_space_steering::PhaseSpaceSteering, cwru_wsn_steering::SteeringBase)

	namespace phase_space_steering {
		const double pi = 3.1415926;
		
		PhaseSpaceSteering::PhaseSpaceSteering() {}

		void PhaseSpaceSteering::initialize(ros::NodeHandle nh_) {
			nh_.param("k_psi", k_psi, 20.0); 
			nh_.param("k_v", k_v, 1.0);
			nh_.param("omega_cmd_sat", omega_cmd_sat, 2.0);
			nh_.param("phase_space_slope",phase_space_slope, -1.0);

			ROS_INFO("Phase Space Steering initialized");
		}

		double PhaseSpaceSteering::psiOfD(double d) {
			//map desired (relative) as a function of displacement from path
			//slope should be negative; large magnitude is more aggressive
			//sign of psi: positive if actual psi is CCW relative to path psi
			double psi = phase_space_slope * d;	
			if (psi > pi/2.) {
				psi = pi/2.;
			} else if (psi < -pi/2.) {
				psi = -pi/2.;
			}
			return psi;
		}

		void PhaseSpaceSteering::computeVelocities(double x_PSO, double y_PSO, double psi_PSO, double x_des, double y_des, double v_des, double psi_des, double rho_des, double &v, double &omega) {
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
			double deltaPsiPSOtoPath = psi_PSO-psi_des;
			//std::cout << "psi_PSO " << psi_PSO << " " << "psi_des " << psi_des << std::endl;
			while (deltaPsiPSOtoPath > pi)
				deltaPsiPSOtoPath -= 2*pi;
			while (deltaPsiPSOtoPath < -pi)
				deltaPsiPSOtoPath += 2*pi;

			//get the phase space mapping of the desired delta psi
			//for the current offset d
			double dPsiMappedToPath = psiOfD(d);

			//compute difference between heading error and ideal
			//heading error
			deltaPsi = dPsiMappedToPath - deltaPsiPSOtoPath;
			double omega_cmd = k_psi*deltaPsi + v*rho_des;
			if(omega_cmd > omega_cmd_sat) {
				omega = omega_cmd_sat;
			} else if(omega_cmd < -omega_cmd_sat) {
				omega = -omega_cmd_sat;
			} else {
				omega = omega_cmd;
			}
		}
	};
