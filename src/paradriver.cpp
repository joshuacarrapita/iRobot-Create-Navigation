#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include <cstdlib>
#include "paradriver.hpp"

#define MAXSPEED 15.0
#define MINSPEED 0.3
#define CORRECTIONMARGIN 3.0
#define WHEELBASE 260		//mm
#define WHEELRADIUS 33		//mm
#define RADIANSPERROTATION 5.5
#define DISTPERROTATION 181.5	//mm

#define ACCELTIME 30
#define THETACORRECTION 1.0
#define DISTTOLERANCE 50	//mm
#define ANGLETOLERANCE 1.5
#define KP 0.20

using namespace std;


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
	ROS_INFO("encoders reset");
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


double DiffDriving::calcAngle(double x, double y)
{
	if (x == 0) {
		if (y >= 0) {
			return 0.0;
		} else {
			return 180.0;
		}
	}
	int signer = 1;
	if (x < this->xcord) { signer = -1; }
	return signer * (acos(y/sqrt(x*x+y*y)) * (180/M_PI));
}

double DiffDriving::calcDist(double x, double y)
{
        return sqrt((x - this->xcord)*(x - this->xcord)+(y - this->ycord)*(y - this->ycord));
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
		double deltaTheta = thetaDiff(distL, distR);
		this->xcord += dist * sin((theta*M_PI/180) + deltaTheta/2);
		this->ycord += dist * cos((theta*M_PI/180) + deltaTheta/2);
		this->theta += deltaTheta * 180/M_PI * THETACORRECTION;
		this->encLeft = msg->encoderLeft;
		this->encRight = msg->encoderRight;
	}
}

//Turn to specific absolute angle
void DiffDriving::turnAbs(double targetTheta)
{
	ros::spinOnce();
	double err = targetTheta - theta;
	double speed;
	ROS_INFO("turning to heading %lf", targetTheta);
	while (abs(err) > ANGLETOLERANCE) {
		//ROS_INFO("err: %lf", err);
		ros::spinOnce();
		err = targetTheta - theta;
		speed = min(MAXSPEED, abs(err * KP));
		if (theta > targetTheta) {
			//ROS_INFO("turning left");
			this->pdsrv.request.left = -speed;
			this->pdsrv.request.right = speed;
		} else {
			//ROS_INFO("turning right");
			this->pdsrv.request.left = speed;
			this->pdsrv.request.right = -speed;
		}
		diffDrive.call(pdsrv);
	}
	ROS_INFO("stopping...");
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
	this->stop();
}


//Turn specific relative angles with P-Control via Odometer
void DiffDriving::turn(double targetTheta)
{
	ros::spinOnce();
	double goal = theta + targetTheta;
	double err = targetTheta;
	double speed;
	//double accel = ACCELTIME;
	while (abs(err) > ANGLETOLERANCE) {
		//ROS_INFO("turning with err: %lf", err);
		ros::spinOnce();
		err = goal - theta;
		speed = min(MAXSPEED, abs(err * KP));
		if (theta > goal) {
			//turn left
			this->pdsrv.request.left = -speed;
			this->pdsrv.request.right = speed;
		} else {
			//turn right
			this->pdsrv.request.left = speed;
			this->pdsrv.request.right = -speed;
		}
		diffDrive.call(pdsrv);
		//if (accel > 0) {accel--;}
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
int DiffDriving::drive(int mode, double x, double y, double targetTheta, int stay)
{
	ros::spinOnce();
	double errX = x - this->xcord;
	double errY = y - this->ycord;
	//double err = (errX + errY)/2;
	//double vecAngle = calcAngle(x,y);
	double err = calcDist(x, y);
	switch(mode)
	{
		case 0:
			double speed;
			turnAbs(calcAngle(x - this->xcord, y - this->ycord));
			while (err > DISTTOLERANCE) {
				ros::spinOnce();
				err = calcDist(x, y);
				speed = min(MAXSPEED, (err * KP + MINSPEED));
				this->pdsrv.request.left = speed;
				this->pdsrv.request.right = speed;
				diffDrive.call(pdsrv);
			}
			this->stop();
			ros::Duration(0.5).sleep();
			ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
			if (stay) { return 1; }
			this->turnAbs(targetTheta);
			return 1;
	}
	return 0;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "diffdriving");
	DiffDriving driver;

	driver.drive(0, 500, 500, 0, 0);
	driver.drive(0, 1, 1, 0, 0);

	/*
	ros::spinOnce();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
	driver.turn(-90);
	driver.moveDist(0.3);
	ros::Duration(0.5).sleep();
	ros::spinOnce();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
	driver.turn(180);
	driver.moveDist(0.3);
	ros::spinOnce();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", driver.xcord, driver.ycord, driver.theta, driver.encLeft, driver.encRight);
	*/

	/* driver.drive(0, 0, 1000, 0, 1);
	driver.drive(0, 1000, 1000, 0, 1);
	driver.drive(0, 1000, 0, 0, 1);
	driver.drive(0, 0, 0, 0, 0); */
	/*
	for (int i = 0; i < 2; i++) {
		driver.turnAbs(-180);
		ros::Duration(0.5).sleep();
		driver.turnAbs(0);
		ros::Duration(0.5).sleep();
	}
	*/
	/*
	driver.turn(180);
	ros::Duration(0.5).sleep();
	driver.turn(180);
	ros::Duration(0.5).sleep();
	driver.turnAbs(0);
	driver.stop();
	*/


	return 0;
}
