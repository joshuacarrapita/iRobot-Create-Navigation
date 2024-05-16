#ifndef PARADRIVER_H
#define PARADRIVER_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

class DiffDriving
{
	public:
		void stop();
		double encoderToDist(double encDiff);
		double centDist(double distLeft, double distRight);
		double thetaDiff(double distLeft, double distRight);
		void odometer(const create_fundamentals::SensorPacket::ConstPtr& msg);
		void turn(double x, double y);
		void smoothTurn(double x, double y, double targetTheta);
		int drive(int mode, double x, double y, double targetTheta);

		ros::NodeHandle n;
		ros::ServiceClient diffDrive;
		create_fundamentals::DiffDrive pdsrv;
		ros::Subscriber pdsub;
		double encLeft;
		double encRight;
		double xcord;
		double ycord;
		double theta;
}

#endif
