#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/GridCells.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <list>
#include <harlie_obstacle_planner/costnode.h>

//Odd numbers ONLY
#define WALL_EXPANSION 21
#define WALL_FACTOR 1

#define MAP_SIZE 4000
#define RESOLUTION .05

#define OPENLIST_EXPAND 20


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

//CostNode nodes[4000][4000];
bool isWall[MAP_SIZE][MAP_SIZE];
float wallCosts[MAP_SIZE][MAP_SIZE];
float gradCosts[MAP_SIZE][MAP_SIZE];
float wallExpand[WALL_EXPANSION][WALL_EXPANSION];

nav_msgs::Odometry odom;
bool isOdomNew;
bool isInit = false;

list<CostNode> nodes(0);


CostNode FindHighestNeighbor(int x, int y)
{
    int currentCost = 0;
    int currentX = 0;
    int currentY = 0;
    if(x-1>0)
    {
        if(gradCosts[x-1][y]>currentCost)
        {
            currentCost=gradCosts[x-1][y];
            currentX = x-1;
            currentY = y;
        }
    }
    if(x+1<MAP_SIZE)
    {
        if(gradCosts[x+1][y]>currentCost)
        {
            currentCost=gradCosts[x+1][y];
            currentX = x+1;
            currentY = y;
        }
    }
    if(y-1>0)
    {
        if(gradCosts[x][y-1]>currentCost)
        {
            currentCost=gradCosts[x][y-1];
            currentX = x;
            currentY = y-1;
        }
    }

    if(y+1<MAP_SIZE)
    {
        if(gradCosts[x][y+11]>currentCost)
        {
            currentCost=gradCosts[x][y+1];
            currentX = x;
            currentY = y+1;
        }
    }
    CostNode out;
    out.x = currentX;
    out.y = currentY;
    out.cost = currentCost;
    return out;
}

void FixGradCost(int x, int y)




void _init()
{
    //clear maps
    for(int x = 0; x < MAP_SIZE ; x++)
    {
        for(int y = 0 ; y < MAP_SIZE ; y++)
        {
            isWall[x][y]=false;
            wallCosts[x][y]=0;
            gradCosts[x][y]=-1;
        }
    }
    //Generate the wall overlay

    int high = ( WALL_EXPANSION - 1 ) / 2;
    int low = -1 * high;

    for(int x = low; x < high; x++)
    {
        for(int y = low; y < high; y++)
        {
           wallExpand[x+( WALL_EXPANSION - 1 ) ][y+( WALL_EXPANSION - 1 )]=1/(abs(x)+abs(y)+1)*WALL_FACTOR;
        }
    }

    //Make a line behind the robot with infinity cost.........
    //Make all the points one pixel infront of the line in the open list
    //These new one forward pixels should have teh correct costs!!!!!!!
    tf::getYaw(odom.pose.pose.orientation);
}

void AddToWallCost(int xR, int yR)
{
    int high = ( WALL_EXPANSION - 1 ) / 2;
    int low = -1 * high;
    for(int x = low; x < high; x++)
    {

        for(int y = low; y < high; y++)
        {
            //This could be condesned.  Basicly make sure we are in the bounds of the map otherwise disregard
           if(xR+x>=0)
           {
               if(xR+x<MAP_SIZE)
               {
                   if(yR+y>=0)
                   {
                        if(yR+y<MAP_SIZE)
                       {
                            wallCosts[xR+x][yR+y] = wallCosts[xR+x][yR+y] + wallExpand[x+( WALL_EXPANSION - 1 ) ][y+( WALL_EXPANSION - 1 )];
                        }
                    }
               }
           }
        }
    }

    isWall[xR][yR]=true;

}

int ConvertFromOdomToArray(float meter)
{
    return ((int)(meter/RESOLUTION - (meter%RESOLUTION)/RESOLUTION))+(int)MAP_SIZE/2;
}

void map_Callback(const nav_msgs::GridCells& msg)
{
    ROS_INFO("Received A gridcell");
    if(!isInit)
    {
        if(isOdomNew)
        {
            _init();
        }
    }


    ROS_INFO("Makeing the new wall map");
    //If we get here then the code has been initalized and the main loop can run.  The initalization makes the starting map.

    //Generate the wall cost map
    bzero(&wallCosts,sizeof(float)*MAP_SIZE*MAP_SIZE);
    bzero(&isWall,sizeof(bool)*MAP_SIZE*MAP_SIZE);
    for(unsigned int i=0; i<msg.cells.size();i++)
    {
        int wallX = ConvertFromOdomToArray(((geometry_msgs::Point)msg.cells[i]).x);
        int wallY = ConvertFromOdomToArray(((geometry_msgs::Point)msg.cells[i]).y);
        AddToWallCost(wallX,wallY);
        /* POSSIBLE UPGRADE
         Requires some type of hash
        if(!isWall[wallX][wallY])
        {
             AddToWallCost(wallX,wallY);
            //If this spot is a new wall,
        }
        if(Wall Removed)
        {
            RemoveFromWallCost(wallX,wallY)
          }

        */
    }


    ROS_INFO("Wallmap made starting to plan");


    int roboX = ConvertFromOdomToArray(odom.pose.pose.position.x);
    int roboY = ConvertFromOdomToArray(odom.pose.pose.position.y);


    int numToPullOffList = OPENLIST_EXPAND;
    if(nodes.size()<OPENLIST_EXPAND)
    {
        numToPullOffList = nodes.size();
    }
    for(int popping = 0; popping<numToPullOffList;popping++)
    {
        CostNode temp = nodes.front();
        nodes.pop_front();
        temp.cost =
    }





    //Step OpenStatesForward to  + 4 meters of current robot state



    //Search for lowest cost in the set of open sets

    lenght = XOpen.size()







}


void loc_Callback(const nav_msgs::Odometry& msg)
{
    odom = msg;
    isOdomNew=true;
}

int main(int argc, char** argv)
{
    isInit = false;
    isOdomNew = false;

    ros::init(argc, argv, "harlie_obstacle_planner");
    Client client("move_base",true);
    client.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x=101;
    goal.target_pose.pose.position.y=0;
    goal.target_pose.pose.position.z=0;
    geometry_msgs::Quaternion quad;
    quad = tf::createQuaternionMsgFromYaw(0.0);
    goal.target_pose.header.frame_id = "/odom";
    ROS_INFO("Harlie Obstacle Planner is spinning");
    goal.target_pose.pose.orientation = quad;
    client.sendGoal(goal);
    ROS_INFO("Harlie Obstacle Planner is spinning");
    //client.waitForResult(ros::Duration(1,1));

    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("/move_base/global_costmap/inflated_obstacles", 2, map_Callback);
    ros::Subscriber loc_sub = n.subscribe("/odom",1,loc_Callback);

    ROS_INFO("Harlie Obstacle Planner is spinning");
    ros::spin();
}






/*




namespace harlie_obstacle_planner {

  HarlieObstaclePlanner::HarlieObstaclePlanner()
  : costmap_ros_(NULL), initialized_(false){}

  HarlieObstaclePlanner::HarlieObstaclePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void HarlieObstaclePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_ros_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      costmap_ros_->getCostmapCopy(costmap_);
      world_model_ = new base_local_planner::CostmapModel(costmap_);
      //we'll get the parameters for the robot radius from the costmap we're associated with
      inscribed_radius_ = costmap_ros_->getInscribedRadius();
      circumscribed_radius_ = costmap_ros_->getCircumscribedRadius();
      footprint_spec_ = costmap_ros_->getRobotFootprint();

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }





  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double HarlieObstaclePlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }
    //if we have no footprint... do nothing
    if(footprint_spec_.size() < 3)
      return -1.0;

    //build the oriented footprint
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    std::vector<geometry_msgs::Point> oriented_footprint;
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_pt.y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }

    geometry_msgs::Point robot_position;
    robot_position.x = x_i;
    robot_position.y = y_i;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);
    return footprint_cost;
  }


  bool HarlieObstaclePlanner::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ros_->getCostmapCopy(costmap_);

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 1.0;
    double dScale = 0.01;

    while(!done)
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = start_yaw;
        ROS_WARN("The carrot planner could not find a valid plan for this goal");
        break;
      }
      target_x = start_x + scale * diff_x;
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

      double footprint_cost = footprintCost(target_x, target_y, target_yaw);
      if(footprint_cost >= 0)
      {
          done = true;
      }
      scale -=dScale;
    }

    plan.push_back(start);
    geometry_msgs::PoseStamped new_goal = goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

    new_goal.pose.position.x = target_x;
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    return (done);
  }

};

*/

