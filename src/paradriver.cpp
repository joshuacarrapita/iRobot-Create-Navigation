#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include <cstdlib>
#include "paradriver.hpp"

#define MAXSPEED 15.0
#define MINSPEED 0.3
#define CORRECTIONMARGIN 3.0
#define WHEELBASE 260	//mm
#define WHEELRADIUS 33	//mm
#define RADIANSPERROTATION 5.5
#define DISTPERROTATION 181.5	//mm

#define THETACORRECTION 0.97
#define ANGLETOLERANCE 2.0
#define KP 0.20

using namespace std;

//ros::ServiceClient diffDrive;
//create_fundamentals::DiffDrive pdsrv;

DiffDriving::DiffDriving()
{
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	pdsub = n.subscribe("sensor_packet", 1, &DiffDriving::odometer, this);
	encLeft = 0;
	encRight = 0;
	xcord = 0;
	ycord = 0;
	theta = 0;
	this->resetOdometer();
	this->odomInit();
}

void DiffDriving::stop()
{
	this->pdsrv.request.left = 0;
	this->pdsrv.request.right = 0;
	diffDrive.call(pdsrv);
}

void DiffDriving::resetOdometer()
{
	ros::ServiceClient resetEncoders = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
	create_fundamentals::ResetEncoders srv;
	resetEncoders.call(srv);
	this->xcord = 0;
	this->ycord = 0;
	this->theta = 0;
	ROS_INFO("encoders reset...");
}

int DiffDriving::odomInit()
{
	ROS_INFO("initialising odometer");
	for (int i = 0; i < 10; i++) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	this->odomReady = 1;
	return 1;
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
	return (distLeft - distRight)/WHEELBASE;
}


//continuously calculates x,y,theta of robot
void DiffDriving::odometer(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	if (!odomReady) {
		this->encLeft = msg->encoderLeft;
		this->encRight = msg->encoderRight;
		ROS_INFO("encoderLeft: %lf encoderRight: %lf", msg->encoderLeft, msg->encoderRight);
	} else {
		double distL = encoderToDist(msg->encoderLeft - encLeft);
		double distR = encoderToDist(msg->encoderRight - encRight);
		double dist = centDist(distL, distR);
		double deltaTheta = thetaDiff(distL, distR) * (180/M_PI);
		this->xcord += dist * sin(theta + deltaTheta/2);
		this->ycord += dist * cos(theta + deltaTheta/2);
		this->theta += deltaTheta * THETACORRECTION;
		this->encLeft = msg->encoderLeft;
		this->encRight = msg->encoderRight;
	}
}

//Turn specific angles with P-Control via Odometer
void DiffDriving::turn(double targetTheta)
{
	ros::spinOnce();
	double goal = theta + targetTheta;
	double err = goal - theta;
	double speed;
	while (abs(err) > ANGLETOLERANCE) {
		//ROS_INFO("turning with err: %lf",err);
		ros::spinOnce();
		err = abs(goal) - abs(theta);
		speed = min(MAXSPEED, (err * KP + MINSPEED));
		if (theta > goal) {
			this->pdsrv.request.left = -speed;
			this->pdsrv.request.right = speed;
		} else {
			this->pdsrv.request.left = speed;
			this->pdsrv.request.right = -speed;
		}
		diffDrive.call(pdsrv);
	}
	ROS_INFO("stopping...");
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
	this->stop();
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
	return 0;
}

void DiffDriving::testdrive(double leftspeed, double rightspeed)
{
	this->pdsrv.request.left = leftspeed;
	this->pdsrv.request.right = rightspeed;
	diffDrive.call(pdsrv);
}

void DiffDriving::moveDist(double dist)
{
	testdrive(6.35, 6.35);
	ros::Duration(dist*5).sleep();
	stop();
	return;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "diffdriving");
	DiffDriving driver;
	//driver.odomInit();
	for (int i = 0; i < 8; i++) {
		driver.turn(-45);
		ros::Duration(0.5).sleep();
	}
	driver.stop();
	/*
	ros::Duration(1.0).sleep();
	driver.turn(-360);
	driver.stop();
	ros::spinOnce();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
	ros::NodeHandle n;
	DiffDriving driver;
	ros::Subscriber sub = n.subscribe("sensor_packet", 1, getEncVals);
	*/
	return 0;
}
