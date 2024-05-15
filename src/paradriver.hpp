#ifndef PARADRIVER_H
#define PARADRIVER_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

class DiffDriving
{
	public:
		void stop();
		void getEncVals();
		int drive(int mode, double x, double y, double theta);
		ros::NodeHandle n;
		ros::ServiceClient diffDrive;
		create_fundamentals::DiffDrive pdsrv;
		ros::Subscriber pdsub;
		double encLeft;
		double encRight;
}

#endif
