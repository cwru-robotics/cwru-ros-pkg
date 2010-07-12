#include <playsound_recovery/playsound_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this class as a RecoveryBehavior plugin
PLUGINLIB_REGISTER_CLASS(PlaySoundRecovery, playsound_recovery::PlaySoundRecovery, nav_core::RecoveryBehavior)

	namespace playsound_recovery {
		PlaySoundRecovery::PlaySoundRecovery(): global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false), player_() {}

		void PlaySoundRecovery::initialize(std::string name, tf::TransformListener* tf,
				costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
			if(!initialized_){
				name_ = name;
				tf_ = tf;
				global_costmap_ = global_costmap;
				local_costmap_ = local_costmap;

				//get some parameters from the parameter server
				ros::NodeHandle private_nh("~/" + name_); 

				initialized_ = true;
			}
			else{
				ROS_ERROR("You should not call initialize twice on this object, doing nothing");
			}
		}
		void PlaySoundRecovery::runBehavior() {
			if(!initialized_){
				ROS_ERROR("This object must be initialized before runBehavior is called");
				return;
			}

			if(global_costmap_ == NULL || local_costmap_ == NULL){
				ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
				return;
			}
			sound_play::Sound s = player_.voiceSound(std::string("Please move out from in front of the robot."));
			s.play();
			ROS_DEBUG("Played a PlaySoundRecovery sound");
		}

	};
