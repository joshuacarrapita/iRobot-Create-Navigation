#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"
#include <cstdlib>

#define TURNSPEED 3.2
#define DRIVESPEED 6.4
#define RASPEED 3.25
#define TASPEED 3.25

ros::ServiceClient::diffDrive;
create_fundamentals::DiffDrive ddsrv;

Moving::Moving()
{
}

void Moving::stop()
{
	ddsrv.request.left = 0;
	ddsrv.request.left = 0;
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
	double anglemapped (abs(angle)*4.0)/180;
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
	ros::Duration(2.0).sleep();
	stop();
	return;
}

void Moving::turnAround()
{
	stop();
	move(TASPEED, -TASPEED);
	ros::Duration(4.0).sleep();
	return;
}

void Moving::moveDist(double dist)
{
	move(DRIVESPEED, DRIVESPEED);
	ros::Duration(dist*5).sleep();
	stop();
	return;
}

int main(int argc, char **argv)
{
	Moving mover;
	mover.moveDist(1);
	mover.turnAround();
	mover.moveDist(1);
	mover.turnAround();
	return 0;
}
