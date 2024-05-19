#ifndef PARADRIVER_H
#define PARADRIVER_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

class DiffDriving
{
	public:
		void stop();
		void resetOdometer();
		int odomInit();
		double calcAngle(double x, double y);
		double calcDist(double x, double y);
		double encoderToDist(double encDiff);
		double centDist(double distLeft, double distRight);
		double thetaDiff(double distLeft, double distRight);
		void odometer(const create_fundamentals::SensorPacket::ConstPtr& msg);
		void turnAbs(double targetTheta);
		void turn(double targetTheta);
		void smoothTurn(double x, double y, double targetTheta);
		int drive(int mode, double x, double y, double targetTheta, int stay);
		void testdrive(double leftspeed, double rightspeed);
		//void turn(double angle);
		void moveDist(double dist);

		ros::NodeHandle n;
		ros::ServiceClient diffDrive;
		create_fundamentals::DiffDrive pdsrv;
		ros::Subscriber pdsub;
		int odomReady;
		double encLeft;
		double encRight;
		double xcord;
		double ycord;
		double theta;
		DiffDriving();
};

#endif
