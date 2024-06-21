#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include "red_fundamentals/StopMsg.h"
#include <cstdlib>
#include "paradriver.hpp"

#define MAXSPEED 10.0
#define MAXTURNSPEED 8.0
#define MINSPEED 0.5
#define CORRECTIONMARGIN 3.0
#define WHEELBASE 0.260		//m
#define WHEELRADIUS 0.033	//m
#define RADIANSPERROTATION 5.5
#define DISTPERROTATION 0.1815	//m

#define ACCELTIME 30
#define THETACORRECTION 0.98
#define DISTTOLERANCE 0.01	//m
#define ANGLETOLERANCE 0.5	//deg
#define ENCTOLERANCE 0.1	//rad

#define KPTURN 0.10
#define KDTURN 0.95
#define KP 0.20		//proportional gain factor
#define KD 0.90		//derivative gain factor

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
	clock = ros::Time::now().toSec();
	this->resetOdometer();
	this->odomInit();
}

bool stop_check(){

	red_fundamentals::StopMsg::ConstPtr stop_message = ros::topic::waitForMessage<red_fundamentals::StopMsg>("stop_topic", ros::Duration(0.2));

	if (stop_message && stop_message->content == "STOP") return true;
	return false;

}


//stop motors
void DiffDriving::stop()
{
	this->pdsrv.request.left = 0;
	this->pdsrv.request.right = 0;
	diffDrive.call(pdsrv);
}

//reset encoders and odometry values to 0
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

//initialise the odometer by making sure we get correct values
int DiffDriving::odomInit()
{
	ROS_INFO("initialising odometer");
	for (int i = 0; i < 10; i++) {
		ros::spinOnce();
		//ros::Duration(0.1).sleep();
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

//calculate euclidian distance between current position and given point
double DiffDriving::calcDist(double x, double y)
{
        return sqrt((x - this->xcord)*(x - this->xcord)+(y - this->ycord)*(y - this->ycord));
}

//calculate euclidian distance to relative given point
double DiffDriving::calcRelDist(double x, double y)
{
        return sqrt(x*x + y*y);
}



//returns distance in m
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
		this->clock = ros::Time::now().toSec();
	}
}

//Turn to specific absolute angle with PD-Control via odometer
void DiffDriving::turnAbs(double targetTheta)
{
	ros::spinOnce();
	double err = abs(targetTheta - theta);
	double speed;
	double lasterr = err;
	double dt;
	double derr;
	//ROS_INFO("turning to heading %lf", targetTheta);
	while (err > ANGLETOLERANCE) {
		//ROS_INFO("err: %lf", err);
		dt = ros::Time::now().toSec() - this->clock;
		ros::spinOnce();
		err = abs(targetTheta - theta);
		derr = lasterr - err;
		lasterr = err;
		speed = min(MAXSPEED, err * KP  + KD * (derr/dt)) + MINSPEED;
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
	//ROS_INFO("stopping...");
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
	this->stop();
}


//Turn specific relative angles with PD-Control via odometer
void DiffDriving::turn(double targetTheta)
{
	ros::spinOnce();
	double goal = theta + targetTheta;
	double err = abs(targetTheta);
	double speed;
	double lasterr = err;
	double dt;
	double derr;
	while (err > ANGLETOLERANCE) {
		//ROS_INFO("turning with err: %lf", err);
		dt = ros::Time::now().toSec() - this->clock;
		lasterr = err;
		ros::spinOnce();
		err = abs(goal - theta);
		derr = lasterr - err;
		ROS_INFO("Differential component: %lf with derr: %lf and dt: %lf\n lasterr: %lf | err: %lf", KD * (derr/dt), derr, dt, lasterr, err);
		speed = min(MAXTURNSPEED, err * KP + KD * (derr/dt)) + MINSPEED;
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
	}
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
	this->stop();
}

//PD-control func driving straight for certain distances
void DiffDriving::moveDistEnc(double dist)
{
	ros::spinOnce();
	double speedL;
	double speedR;
	double setpointL = dist*32 + this->encLeft;
	double setpointR = dist*32 + this->encRight;
	double errL = setpointL - this->encLeft;
	double errR = setpointR - this->encRight;
	double lasterrL = errL;
	double lasterrR = errR;
	double dt;
	double derrL;
	double derrR;
	while (abs(errL) + abs(errL) > ENCTOLERANCE) {
		/*
		if (stop_check()) {
			break;
		}
		*/
		dt = ros::Time::now().toSec() - this->clock;
		ros::spinOnce();
		errL = setpointL - this->encLeft;
		errR = setpointR - this->encRight;
		derrL = abs(lasterrL) - abs(errL);
		derrR = abs(lasterrR) - abs(errR);
		ROS_INFO("errL: %lf | errR: %lf | derrL: %lf | derrR: %lf",errL,errR,derrL,derrR);
		speedL = errL * KP * 5 + KD * (derrL/dt) * 2;
		speedR = errR * KP * 5 + KD * (derrR/dt) * 2;
		if (errL < 0) {
			speedL -= MINSPEED;
		} else {
			speedL += MINSPEED;
		}
		if (errR < 0) {
			speedR -= MINSPEED;
		} else {
			speedR += MINSPEED;
		}
		pdsrv.request.left = speedL;
		pdsrv.request.right = speedR;
		lasterrL = errL;
		lasterrR = errR;
		diffDrive.call(pdsrv);
	}
	this->stop();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
	return;
}

/*Idea:
 *mode 0: Turn to face goalpoint, Drive until at goalpoint, Turn to meet targetTheta
 *mode 1: Turn to face relative goalpoint, drive until reached, turn to meet targetTheta
 *mode 2: Do a smooth turn until goalpoint is hit, adjust theta to meet targetTheta
*/
int DiffDriving::drive(int mode, double x, double y, double targetTheta, int stay)
{
	ros::spinOnce();
	switch(mode)
	{
		case 0: {
			double speed;
			turnAbs(calcAngle(x - this->xcord, y - this->ycord));
			double err = calcDist(x, y);
			this->moveDistEnc(err);	//only calculate euclidean distance once then drive straight for that amount of distance
			ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
			if (stay) { return 1; }
			this->turnAbs(targetTheta);
			ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
			return 1;
		}
		case 1: {
			double speed;
			turnAbs(calcAngle(x, y));
			double err = calcRelDist(x, y);
			this->moveDistEnc(err);
			ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
			if (stay) { return 1; }
			this->turnAbs(targetTheta);
			ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
			return 1;
		}
		//idea for smooth driving:
		/*-Use local coordinate system with y axis always being robot heading
		  -Each step calculate delta theta,x,y to target point
		  -First drive according to delta theta (=angle of vector to target point from current robot position)
			->if d_theta < 0 -> turn left by using higher speed for right wheel
			->if d_theta > 0 -> turn right by using higher speed for left wheel
		  -The delta of the speeds should be governed with a factor derived from delta theta
		  -Then drive according to delta x and y
			->

		*/
	}
	return 0;
}

void driveWithCorrection(double x, double y, double targetTheta)
{
	for (int i = 1; i < 6; i++) {
		i = i;
	}
}

void DiffDriving::correct_odometry_location_error(double x_dest, double y_dest, double x_center_rel_act, double y_center_rel_act)
{
	//x_center_rel_act is the relative x coordinate of the actual center. The x,y-coordinates should always be the destination minus the actual offset from the destination.
	if(x_center_rel_act != 0.0) {
		this->xcord = x_dest - x_center_rel_act;
	}
	if(y_center_rel_act != 0.0) {
		this->ycord = y_dest - y_center_rel_act;
	}
}

void DiffDriving::correct_odometry_angle_error(double x_dest, double y_dest, double x_center_rel_act, double y_center_rel_act)
{
	// the angle correction is the supposed angle toward the center points minus the actual angle towards the center point
	if (x_center_rel_act != 0.0 && y_center_rel_act != 0.0)
	{
		double theta_dest = calcAngle(x_dest, y_dest);
		double theta_dest_act = calcAngle(this->xcord + x_center_rel_act, this->ycord + y_center_rel_act);
		this->theta += theta_dest - theta_dest_act;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "diffdriving");
	DiffDriving driver;
	dir di;
	for (int i = 0; i < 16; i++) {
		driver.turn(90);
		ros::Duration(0.3).sleep();
	}

	return 0;
}
