#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"

void laserSensObstacle(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//TODO: implement function that sets some variable/does some thing when an obstacle is detected
	for (int i = 0; i < 90; i++){
		//ROS_INFO("%f", msg->ranges[135+1]);
		if (msg->ranges[135+i] < 0.2) {
			ROS_INFO("Obstacle detected!");
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wanderer");
	ros::NodeHandle n;
	ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive"); 
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserSensObstacle);
	create_fundamentals::DiffDrive srv;
	ROS_INFO("Starting wanderer mode");
	srv.request.left = 10;
	srv.request.right = 10;
	diffDrive.call(srv);

	ros::Duration(5.0).sleep();

	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);

	ros::spin();
	return 0;
}
