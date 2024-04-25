#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/WallInfo.h"

red_fundamentals::WallInfo winfo_msg;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("ONE LASER READING: [");
  for (int i = 0; i < msg->ranges.size(); i++) {
  	ROS_INFO("%f", msg->ranges[i]);
  }
  ROS_INFO("]\n");
  winfo_msg.dist_wall_front = 110;
  winfo_msg.dist_wall_right = 111;
  winfo_msg.dist_wall_back = 112;
  winfo_msg.dist_wall_left = 113;
  winfo_msg.angle_wall_front = 90;
  winfo_msg.angle_wall_right = 91;
  winfo_msg.angle_wall_back = 92;
  winfo_msg.angle_wall_left = 93;
  ROS_INFO("WallInfo msg: %f, %f, %f, %f, %f, %f, %f, %f\n", winfo_msg.dist_wall_front, winfo_msg.dist_wall_right, winfo_msg.dist_wall_back, winfo_msg.dist_wall_left, winfo_msg.angle_wall_front, winfo_msg.angle_wall_right, winfo_msg.angle_wall_back, winfo_msg.angle_wall_left);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_info_talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<red_fundamentals::WallInfo>("wall_info_topic", 100);
  ros::Subscriber sub = n.subscribe("scan_filtered", 100, laserCallback);
  
  ros::Rate loop_rate(10);
  while (ros::ok()){
  	ros::spinOnce();

	pub.publish(winfo_msg);

	loop_rate.sleep();
  }

  return 0;
}
