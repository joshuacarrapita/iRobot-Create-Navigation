#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include <math.h>

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;
double speed = 5;		//default: 5
double turnaroundDist = 0.2;	//default: 0.2
double turningTime = 1;	//default: 1

void stop() {
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);
}

void turnAround(int direction) {
	int turnspeed = 7;
	if (!direction) {
		turnspeed = -turnspeed;
	}
	stop();
	srv.request.left = turnspeed;
	srv.request.right = -turnspeed;
	diffDrive.call(srv);
	ros::Duration(turningTime).sleep();
	stop();
}
//WARNING:As soon as the odometer value overflows (>2^15) the driver will crash and need to be restarted
void wander(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int minDistIndex;
	int turnDirection = 1;
	double minDist = 1;
	for (int i = msg->ranges.size()/4; i < (msg->ranges.size()/4)*3; i++) {
		if (msg->ranges[i] < minDist) {
			minDist = msg->ranges[i];
			minDistIndex = i;
		}
	}
	if (minDistIndex < msg->ranges.size()/2) {
		turnDirection = 0;
	}
	if (minDist < turnaroundDist) {
		turnAround(turnDirection);
	} else {
		srv.request.left = speed;
		srv.request.right = speed;
		diffDrive.call(srv);
	}
}


int main(int argc, char **argv)
{
	ROS_ERROR("As soon as the odometer value reaches 32768 the driver will crash and need to be restarted!");
	if (argc != 4) {
		ROS_ERROR("Usage: wanderer <speed> <turnaroundDist> <turningTime>");
		ROS_INFO("Setting default values...");
	} else {
		speed = atof(argv[1]);
		turnaroundDist = atof(argv[2]);
		turningTime = atof(argv[3]);
	}
	ros::init(argc, argv, "wanderer");
	ros::NodeHandle n;
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, wander);
	ROS_INFO("Starting wanderer mode");
	ros::spin();
	return 0;
}
