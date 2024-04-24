#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"


void laserSensObstacle(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//TODO: implement function that sets some variable/does some thing when an obstacle is detected

	ROS_INFO("Distance: %f", msg->ranges[msg->ranges.size()/2]);
	for (int i = 0; i < 90; i++){
		//ROS_INFO("%f", msg->ranges[135+1]);
		if (msg->ranges[135+i] < 0.2) {
			//ROS_INFO("Obstacle detected!");
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
	srv.request.left = 7;
	srv.request.right = 7;
	diffDrive.call(srv);
	for (int i = 0; i < 10; i++) {
		diffDrive.call(srv);
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}

	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);

	return 0;
}
