#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"
#include <cstdlib>

#define TURNSPEED 3.2
#define DRIVESPEED 6.35
#define RASPEED 3.2
#define TASPEED 3.15

Moving::Moving() {
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
}

void Moving::stop()
{
	ddsrv.request.left = 0;
	ddsrv.request.right = 0;
	diffDrive.call(ddsrv);
	return;
}

void Moving::move(double lspeed, double rspeed)
{
	ddsrv.request.left = lspeed;
	ddsrv.request.right = rspeed;
	diffDrive.call(ddsrv);
	return;
}

void Moving::turn(double angle)
{
	stop();
	double anglemapped = (abs(angle)*4.0)/180;
	if (angle > 0) {
		move(TURNSPEED, -TURNSPEED);
	} else {
		move(-TURNSPEED, TURNSPEED);
	}
	ros::Duration(anglemapped).sleep();
	stop();
	return;
}

void Moving::turnRA(int direction)
{
	stop();
	if (direction) {
		move(RASPEED, -RASPEED);
	} else {
		move(-RASPEED, RASPEED);
	}
	ros::Duration(2.06).sleep();
	stop();
	return;
}

void Moving::turnAround()
{
	stop();
	move(TASPEED, -TASPEED);
	ros::Duration(4.21).sleep();
	stop();
	return;
}

void Moving::moveDist(double dist)
{
	move(DRIVESPEED, DRIVESPEED);
	ros::Duration(dist*5).sleep();
	stop();
	return;
}

/*int main(int argc, char **argv)
{
	ros::init(argc, argv, "moving");
	ros::NodeHandle n;
	Moving mover;
	//for (int i = 0; i < 16; i++) {
	//	mover.moveDist(1.0);
	//	mover.turnRA(1);
	//	//ros::Duration(0.2).sleep();
	//}
	mover.turnAround();
	mover.moveDist(0.8);
	mover.turnRA(0);
	mover.moveDist(0.8);
	mover.turnRA(1);
	mover.moveDist(0.8);
	mover.turnAround();
	mover.moveDist(0.8);
	mover.turnRA(0);
	mover.moveDist(0.8);
	mover.turnRA(0);
	mover.moveDist(0.8);
	mover.turnRA(1);
	mover.moveDist(0.8);
	return 0;
}*/
