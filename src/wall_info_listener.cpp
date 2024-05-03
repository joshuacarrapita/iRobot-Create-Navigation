#include "ros/ros.h"
#include <cstdlib>
#include "red_fundamentals/WallInfo.h"

void winfoCallback(const red_fundamentals::WallInfo::ConstPtr& msg)
{
	ROS_INFO("I HEARD: %f, %f, %f, %f\n", msg->x_move, msg->y_move, msg->x_dir_prll, msg->y_dir_prll);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wall_info_listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("wall_info_topic", 100, winfoCallback);
	ros::spin();
	return 0;
}
