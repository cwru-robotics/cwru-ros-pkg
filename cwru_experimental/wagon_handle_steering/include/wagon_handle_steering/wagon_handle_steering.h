#ifndef WAGON_HANDLE_STEERING_H_
#define WAGON_HANDLE_STEERING_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <base_local_planner/trajectory_planner_ros.h>

namespace wagon_handle_steering {
  class WagonHandleSteering : public nav_core::BaseLocalPlanner {
    public:
      WagonHandleSteering();
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }

      geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose&  pose2);
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
      double headingDiff(double pt_x, double pt_y, double x, double y, double heading);

      bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
          const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      bool stopped();

      tf::TransformListener* tf_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      ros::Publisher vel_pub_;
      double handle_length_, reorient_dist_, rotate_in_place_dist_, rotate_in_place_head_;
      double tolerance_timeout_, tolerance_trans_, tolerance_rot_;
      //These are just to make it compile right now
      double K_trans_, K_rot_;
      int samples_;
      bool holonomic_;
      //End compile junk
      double max_vel_lin_, max_vel_th_;
      double min_vel_lin_, min_vel_th_;
      double min_in_place_vel_th_, in_place_trans_vel_;
      boost::mutex odom_lock_;
      ros::Subscriber odom_sub_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
      ros::Time goal_reached_time_;
      unsigned int current_waypoint_; 
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      base_local_planner::TrajectoryPlannerROS collision_planner_;
  };
};
#endif
