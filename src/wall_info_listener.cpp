#include "ros/ros.h"
#include <cstdlib>
#include "red_fundamentals/WallInfo.h"

void winfoCallback(const red_fundamentals::WallInfo::ConstPtr& msg)
{
  ROS_INFO("I HEARD: %f, %f, %f, %f, %f, %f, %f, %f\n", msg->dist_wall_front, msg->dist_wall_right, msg->dist_wall_back, msg->dist_wall_left, msg->angle_wall_front, msg->angle_wall_right, msg->angle_wall_back, msg->angle_wall_left);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_info_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("wall_info_topic", 100, winfoCallback);
  ros::spin();
  return 0;
}
