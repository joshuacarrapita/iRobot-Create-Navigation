#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"

:

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_info_talker");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise("wall_info", 100);
  ros::Subscriber sub = n.subscribe("scan_filtered", 100, laserCallback);
  
  ros::Rate loop_rate(10);

  while (ros::ok()){
  	ros::spinOnce();
  }

  return 0;
}
