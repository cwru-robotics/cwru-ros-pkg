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
      double propotionalError;
      double integralError;
      double derivativeError;

      double proportionalGain;
      double integralGain;
      double derivativeGain;
      
      geometry_msgs::Twist limitTwist(const double desired_angular_rate, const double desired_speed);
      bool intersectedWithCircle(const tf::Point& start_p, const tf::Point& robot_p, const tf::Vector3& direction, tf::Point& intersectionPoint);
      bool shouldRotateInPlace(const tf::Point& start, const tf::Point& end, const tf::Pose& current_loc);

      bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
          const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      bool stopped();
      void updateWaypoint(tf::Stamped<tf::Pose>& robot_pose);
      double calcDistanceToGoal(tf::Stamped<tf::Pose>& robot_pose);

      double PIDSteering(double angle_correction);

      double max_rotational_vel_,max_vel_y_,max_vel_x_,acc_lim_th_,acc_lim_x_,acc_lim_y_,min_vel_x_, min_vel_y_,min_in_place_rotational_vel_,min_rotational_vel_,update_time_;

      tf::TransformListener* tf_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      ros::Publisher vel_pub_;
      double handle_length_, reorient_dist_, rotate_in_place_dist_, rotate_in_place_head_, desired_speed_, advance_radius_;
      double tolerance_timeout_, tolerance_trans_, tolerance_rot_;
      //These are just to make it compile right now
      int samples_;
      bool holonomic_;
      //End compile junk

      boost::mutex odom_lock_;
      ros::Subscriber odom_sub_;
      ros::Publisher desired_heading_pub_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
      ros::Time goal_reached_time_;
      unsigned int current_waypoint_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      base_local_planner::TrajectoryPlannerROS collision_planner_;
      bool started_reorienting_;
      int forward_waypoint_check_count_;
      int collision_tries_;
      double vel_decay_;
      double angular_decay_;
  };
};
#endif
