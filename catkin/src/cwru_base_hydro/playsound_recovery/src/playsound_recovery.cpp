/* Copyright (c) 2010, Eric Perko
 * All rights reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <playsound_recovery/playsound_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(playsound_recovery, PlaySoundRecovery, playsound_recovery::PlaySoundRecovery, nav_core::RecoveryBehavior)

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
