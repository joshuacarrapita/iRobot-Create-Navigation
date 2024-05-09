#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>

ros::ServiceClient::diffDrive;
create_fundamentals::DiffDrive ddsrv;

void stop()
{
	ddsrv.request.left = 0;
	ddsrv.request.left = 0;
	diffDrive.call(ddsrv);
	return;
}

