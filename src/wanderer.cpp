#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"

void laserSensObstacle(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//TODO: implement function that sets some variable/does some thing when an obstacle is detected
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wanderer");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserSensObstacle);
	ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	ros::spin();
	return 0;
}
