#ifndef MOVING_NONSTOP_H
#define MOVING_NONSTOP_H
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"

class Moving
{
	public:
		void stop();
		void move(double dist);
		void curve(int direction, double angle, double radius);
		ros::ServiceClient diffDrive;
		create_fundamentals::DiffDrive ddsrv;
		ros::NodeHandle n;
		Moving(); // Constructor
};

#endif

