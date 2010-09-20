#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <string>
#include <list>



std::list<geometry_msgs::Point32> points;
std::string frame_id;
bool recieved_map;
bool updated_map;

void mapOccupancyGridCallback(const  nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  points.clear();
  recieved_map=true;
  updated_map=true;
  frame_id=msg->header.frame_id;
  geometry_msgs::Point32 p;
  for(unsigned int m=0;m<msg->info.height;m++){
    for(unsigned int n=0;n<msg->info.width;n++){
    //currently 100 is the only value used for obsticles in the map
      if(msg->data[m*msg->info.width+n]==100){
          p.x=n*msg->info.resolution+msg->info.origin.position.x;
          p.y=m*msg->info.resolution+msg->info.origin.position.y;
          points.push_back(p);
      }
    }
  }
}


int main(int argc, char *argv[]){
  //boolean switch for if we have ever recieved a map
  recieved_map=false;
  //boolean switch that triggers every time the map is updated
  updated_map=false;
  
  
  //intensity of all points in the cloud
  double intensity=100;
  //rate in Hz the sensor outputs its data
  int sensor_rate=5;
  //height of the sensor in meters
  double height=8.;
  //name of the channel the points are in, for the cloud
  std::string channel_name="intensity";

  //start node
 	ros::init(argc, argv, "map_as_sensor");
 	ros::NodeHandle n;
 	ros::NodeHandle priv_nh_("~"); 
 	
 	// parameters for the node
 	priv_nh_.param("intensity", intensity,100.);
	priv_nh_.param("sensor_rate",sensor_rate, 1);
	priv_nh_.param("channel_name",channel_name, std::string("intensity"));
	priv_nh_.param("height",height, 8.);
 	
  ros::Subscriber sub = n.subscribe("input_map", 1, &mapOccupancyGridCallback);
 
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("map_cloud",1);

  sensor_msgs::PointCloud * sensor_points=NULL;
    //continuously publish the point cloud at a set rate
  ros::Rate r(sensor_rate);
  
  
  while(n.ok()){
    if(recieved_map){
      if(updated_map){
        updated_map=false;
        
        if(sensor_points!=NULL){ delete(sensor_points);}
        
        sensor_points=NULL;
        sensor_points=new sensor_msgs::PointCloud;
        sensor_points->set_points_size(points.size());
        
        sensor_points->set_channels_size(1);
        sensor_points->channels[0].name = channel_name;
        sensor_points->channels[0].set_values_size(points.size());
        
        sensor_points->header.frame_id =frame_id;
        //for everything in list points, set the x and y value to the right stuff, and set intensity to intensity
        int point_number=0;
        for(std::list<geometry_msgs::Point32>::const_iterator iterator = points.begin(), end = points.end(); iterator != end; ++iterator){
          sensor_points->points[point_number].x = (*iterator).x;
          sensor_points->points[point_number].y = (*iterator).y;
          sensor_points->points[point_number].z = height;
          sensor_points->channels[0].values[point_number] = (float)intensity;
         ++point_number;
        }
      }
      
      sensor_points->header.stamp = ros::Time::now();
      cloud_pub.publish(*sensor_points);
    }
    
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

