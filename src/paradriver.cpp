	#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>
#include "paradriver.hpp"

#define MAXSPEED 15.625
#define MINSPEED 0.3
#define CORRECTIONMARGIN 3.0
#define WHEELBASE 260	//mm
#define WHEELRADIUS 33	//mm
#define RADIANSPERROTATION 5.5
#define DISTPERROTATION 181.5	//mm

#define ANGLETOLERANCE 2
#define KP 1.5

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
	this->xcord = 0;
	this->ycord = 0;
	this->theta = 0;
}

int DiffDriving::odomInit()
{
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
	return (distRight - distLeft)/WHEELBASE;
}


//continuously calculates x,y,theta of robot
void DiffDriving::odometer(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	if (!odomReady) {
		this->encLeft = msg->encoderLeft;
		this->encRight = msg->encoderRight;
	} else {
		double distL = encoderToDist(msg->encoderLeft - encLeft);
		double distR = encoderToDist(msg->encoderRight - encRight);
		double dist = centDist(distL, distR);
		double deltaTheta = thetaDiff(distL, distR) * (180/M_PI);
		this->xcord += dist * sin(theta + deltaTheta/2);
		this->ycord += dist * cos(theta + deltaTheta/2);
		this->theta += deltaTheta;
		this->encLeft = msg->encoderLeft;
		this->encRight = msg->encoderRight;
	}
}

void DiffDriving::turn(double targetTheta)
{
	ros::spinOnce();
	double err = targetTheta - theta;
	double speed;
	while (abs(err) > ANGLETOLERANCE)) {
		ros::spinOnce();
		err = targetTheta - theta;
		speed = min(13, err * KP);
		if (targetTheta > theta) {
			this->pdsrv.request.left = -speed;
			this->pdsrv.request.right = speed;
		} else {
			this->pdsrv.request.left = speed;
			this->pdsrv.request.right = -speed;
		}
	}
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

void DiffDriving::turn(double angle)
{
	stop();
	double anglemapped = (abs(angle)*4.0)/180;
	if (angle > 0) {
		testdrive(3.2, -3.2);
	} else {
		testdrive(-3.2, 3.2);
	}
	ros::Duration(anglemapped).sleep();
	stop();
	return;
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
	/*
	for (int t = 0; t < 10; t++) {
		ros::spinOnce();
		ros::Duration(0.2).sleep();
	}
	driver.resetOdometer();
	driver.odomReady = 1;
	*/

	/*
	for (int i = 0; i < 4; i++) {
		ros::spinOnce();
		driver.turn(-90);
		ros::spinOnce();
		ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
		ros::Duration(0.1).sleep();
	}
	for (int i = 0; i < 4; i++) {
		ros::spinOnce();
		driver.turn(90);
		ros::spinOnce();
		ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
		ros::Duration(0.1).sleep();
	}
	*/
	/*
	for (int i = 0; i < 50; i++) {
		ros::spinOnce();
		driver.testdrive(6.35, 6.35);
		ros::spinOnce();
		ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
		ros::Duration(0.1).sleep();
	}
	driver.stop();
	ros::Duration(0.5).sleep();
	for (int i = 0; i < 50; i++) {
		ros::spinOnce();
		driver.testdrive(-6.35, -6.35);
		ros::spinOnce();
		ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
		ros::Duration(0.1).sleep();
	}
	*/
	/*
	while(ros::ok()) {
		ros::spinOnce();
		ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
		ros::Duration(0.2).sleep();
	}
	*/
	driver.turn(90);
	driver.stop();
	ros::spinOnce();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
	/*
	ros::NodeHandle n;
	DiffDriving driver;
	ros::Subscriber sub = n.subscribe("sensor_packet", 1, getEncVals);
	*/
	return 0;
}
