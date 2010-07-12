#ifndef PLAYSOUND_RECOVERY_H
#define PLAYSOUND_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace playsound_recovery {
	class PlaySoundRecovery : public nav_core::RecoveryBehavior {
		public:
			PlaySoundRecovery();
			void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);
			void runBehavior(); 
		private:
			costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
			std::string name_;
			tf::TransformListener* tf_;
			bool initialized_;
			sound_play::SoundClient player_;
	};
};
#endif
