#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>
#include "paradriver.h"

#define MAXSPEED 15.625
#define MINSPEED 0.3
#define CORRECTIONMARGIN 3.0
#define WHEELBASE 260	//mm
#define WHEELRADIUS 33	//mm
#define RADIANSPERROTATION 5.5
#define DISTPERROTATION 181.5	//mm

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive pdsrv;

DiffDriving::DiffDriving()
{
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	pdsub = n.subscribe("sensor_packet", 1, odometer);
	encLeft = 0;
	encRight = 0;
	xcord = 0;
	ycord = 0;
	theta = 0;
}

void DiffDriving::stop()
{
	pdsrv.request.left = 0;
	pdsrv.request.right = 0;
	diffDrive.call(pdsrv);
}

//returns distance in mm
double DiffDriving::encoderToDist(double encDiff)
{
	return (encDiff/RADIANSPERROTATION)*DISTPERROTATION;
}

//takes distance travelled by each wheel and returns distance travelled of the centre of robot
double DiffDriving::centDist(double distLeft, double distRight)
{
	return (distLeft + distRight)/2;
}

//returns difference in theta according to distance travelled by each wheel
double DiffDriving::thetaDiff(double distLeft, double distRight)
{
	return (distRight - distLeft)/WHEELBASE;
}


//continuously calculates x,y,theta of robot
void DiffDriving::odometer(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	double distL = encoderToDist(msg->encoderLeft - encLeft);
	double distR = encoderToDist(msg->encoderRight - encRight);
	double dist = centDist(distL, distR);
	double deltaTheta = thetaDiff(distL, distR);
	xcord += dist * cos(theta + deltaTheta/2);
	ycord += dist * sin(theta + deltaTheta/2);
	theta += deltaTheta;
	encLeft = msg->encoderLeft;
	encRight = msg->encoderRight;
}

void DiffDriving::turn(double x, double y)
{
	return;
}

void DiffDriving::smoothTurn(double x, double y, double targetTheta)
{
	return;
}

/*Idea:
 *mode 0: Turn to face goalpoint, Drive until at goalpoint, Turn to meet targetTheta
 *mode 1: Do a smooth turn until goalpoint is hit, adjust theta to meet targetTheta
*/
int DiffDriving::drive(int mode, double x, double y, double targetTheta)
{
	return;
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
