#include "ros/ros.h"
#include "map.hpp"

int row = 0;
int column = 0;
int orientation = 0;

int main(int argc, char **argv)
{
  if (argc != 4){
	ROS_INFO("Setting default values...");
  } 
  else{
  	row = atoi(argv[1]);
  	column = atoi(argv[2]);
  	orientation = atoi(argv[3]);
  }
  
  ros::init(argc, argv, "localization");
  ros::NodeHandle n;

  // function for saving map
  // Map map = Map();
  
  // start localization algorithm
  
  // create publisher to publish the position
  // TODO check if the server received the message
  ros::Publisher pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);
  red_fundamentals::Pose pose;
    pose.row = row;
    pose.column = column;
    pose.orientation = orientation;
    
    ros::Duration(1.0).sleep();
    pose_pub.publish(pose);
    ROS_INFO("Pose message published");

  return 0;
}

