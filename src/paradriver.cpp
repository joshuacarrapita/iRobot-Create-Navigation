#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>
#include "paradriver.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive pdsrv;

DiffDriving::DiffDriving()
{
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	pdsub = n.subscribe("sensor_packet", 1, getEncVals);
	encLeft = 0;
	encRight = 0;
}

void DiffDriving::stop()
{
	pdsrv.request.left = 0;
	pdsrv.request.right = 0;
	diffDrive.call(pdsrv);
}

void getEncVals(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	encLeft = msg->encoderLeft;
	encRight = msg->encoderRight;
}

int DiffDriving::drive(int mode, double x, double y, double theta)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "diffdriving");
	/*
	ros::NodeHandle n;
	DiffDriving driver;
	ros::Subscriber sub = n.subscribe("sensor_packet", 1, getEncVals);
	*/
	return 0;
}
