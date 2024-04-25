#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "sensor_msgs/LaserScan.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

void center(const red_fundamentals::wall_info::ConstPtr& msg)
{
	//TODO: implement function that uses data from the wall detection service to center the robot within a cell
	
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "centering");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("wall_info", 1, center);
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	//ros::spin();
	return 0;

}
