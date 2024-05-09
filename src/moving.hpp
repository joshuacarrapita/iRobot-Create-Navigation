#ifndef MOVING_H
#define MOVING_H
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"

class Moving
{
	public:
		void stop();
		void move(double lspeed, double rspeed);
		void turn(double angle);
		void turnRA(int direction);
		void turnAround();
		void moveDist(double dist);
		ros::ServiceClient diffDrive;
		create_fundamentals::DiffDrive ddsrv;
		ros::NodeHandle n;
		Moving(); // Constructor
};

#endif
