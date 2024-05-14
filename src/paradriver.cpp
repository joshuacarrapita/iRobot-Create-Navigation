#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>


ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive pdsrv;


