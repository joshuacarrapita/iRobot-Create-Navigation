#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include <math.h>

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;
double speed;
double turnaroundDist;

void stop() {
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);
}

void turnAround() {
	stop();
	srv.request.left = 7;
	srv.request.right = -7;
	diffDrive.call(srv);
	ros::Duration(1.8).sleep();
	stop();
}
void laserSensObstacle(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double minDist = 1;
	for (int i = msg->ranges.size()/3; i < (msg->ranges.size()/3)*2; i++) {
		if (msg->ranges[i] < minDist) {
			minDist = msg->ranges[i];
		}
	}
	ROS_INFO("Distance: %f", minDist);
	if (minDist < turnaroundDist && !isnan(msg->ranges[msg->ranges.size()/2])) {
		turnAround();
	} else {
		srv.request.left = speed;
		srv.request.right = speed;
		diffDrive.call(srv);
	}
}


int main(int argc, char **argv)
{
	if (argc != 3) {
		ROS_ERROR("Usage: wanderer speed turnaroundDist");
	}
	speed = argv[1];
	turnaroundDist = argv[2];
	ros::init(argc, argv, "wanderer");
	ros::NodeHandle n; 
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserSensObstacle);
	ROS_INFO("Starting wanderer mode");
	ros::spin();
	return 0;
}
