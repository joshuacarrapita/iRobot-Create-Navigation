#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include <math.h>

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

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
	ros::Duration(1.7).sleep();
	stop();
}
void laserSensObstacle(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//TODO: implement function that sets some variable/does some thing when an obstacle is detected
	double minDist = 1;
	for (int i = msg->ranges.size()/4; i < (msg->ranges.size()/4)*3; i++) {
		if (msg->ranges[i] < minDist) {
			minDist = msg->ranges[i];
		}
	}
	ROS_INFO("Distance: %f", minDist);
	if (minDist < 0.2 && !isnan(msg->ranges[msg->ranges.size()/2])) {
		turnAround();
	} else {
		srv.request.left = 7;
		srv.request.right = 7;
		diffDrive.call(srv);
	}
	/*
	for (int i = 0; i < 90; i++){
		//ROS_INFO("%f", msg->ranges[135+1]);
		if (msg->ranges[135+i] < 0.2) {
			//ROS_INFO("Obstacle detected!");
		}
	}
	*/
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wanderer");
	ros::NodeHandle n; 
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserSensObstacle);
	ROS_INFO("Starting wanderer mode");
	ros::spin();
	return 0;
}
