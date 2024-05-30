#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include <math.h>

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;
double speed = 7;		//default: 7
double turnaroundDist = 0.15;	//default: 0.15
double turningTime = 0.9;	//default: 0.9

//stop robot
void stop() {
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);
}

//turn left or right depending on direction value
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

//Drive forward until obstacle detected then turn and keep driving
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
