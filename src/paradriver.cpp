#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include "red_fundamentals/StopMsg.h"
#include <cstdlib>
#include "paradriver.hpp"
#include <Eigen/Dense>


#define MAXSPEED 15.0				//max allowed speed for straight motion
#define MAXTURNSPEED 10.0			//max allowed speed for rotational motion
#define MINSPEED 0.4				//min speed to facilitate actual robot movement
#define CORRECTIONMARGIN 3.0
#define WHEELBASE 0.260				//m
#define WHEELRADIUS 0.033			//m
#define RADIANSPERROTATION 5.5			//rad
#define DISTPERROTATION 0.1815			//m
#define ENCPM 31.95				//change in enc values per meter driven

#define ACCELTIME 30
#define DISTCORRECTION 0.95
#define THETACORRECTION 1.0			//correction factor for odometry theta calculation
#define DISTTOLERANCE 0.03			//m
#define ANGLETOLERANCE 1.0			//deg
#define ANGLETOLERANCEREL 0.5			//deg
#define ENCTOLERANCE 0.05			//rad

#define KPTURN 0.5				//proportional gain for turning
#define KDTURN 0.03				//derivative gain for turning
#define KP 1.50					//proportional gain factor for driving straight
#define KD 0.01					//derivative gain factor for driving straight

using namespace std;

enum CoordinateAxis { X = 0, Y = 1, Z = 2};
enum drive_direction { RIGHT = 0, UP = 1, LEFT = 2, DOWN = 3};

DiffDriving::DiffDriving()
{
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	CalculateCenterClient = n.serviceClient<red_fundamentals::CalculateCenterService>("calculate_center_service");
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

	red_fundamentals::StopMsg::ConstPtr stop_message = ros::topic::waitForMessage<red_fundamentals::StopMsg>("stop_topic", ros::Duration(0.1));

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
	return;
}

//initialise the odometer by making sure we get correct values
int DiffDriving::odomInit()
{
	ROS_INFO("initialising odometer");
	for (int i = 0; i < 10; i++) {
		ros::spinOnce();
	}
	this->odomReady = 1;
	return 1;
}

//calculates relative angle of point to robot via dot product of direction vector
double DiffDriving::calcAngle(double x, double y)
{
	/*
	if (x == 0) {
		if (y >= 0) {
			return 0.0;
		} else {
			return 180.0;
		}
	}
	*/
	Eigen::Vector2d yaxis;
	yaxis(X) = 0;
	yaxis(Y) = 1;

	Eigen::Vector2d rel_point;
	rel_point(X) = x;
	rel_point(Y) = y;

	yaxis.normalize();
	rel_point.normalize();

	double dotprod = yaxis.dot(rel_point);
	int signer = 1;
	if (x < 0) { signer = -1; }
	return signer * (acos(dotprod) * (180/M_PI));
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

		double dist = centDist(distL, distR) * DISTCORRECTION;
		double deltaTheta = thetaDiff(distL, distR);
		this->xcord += dist * sin((theta*M_PI/180) + deltaTheta/2);
		this->ycord += dist * cos((theta*M_PI/180) + deltaTheta/2);
		this->theta += deltaTheta * 180/M_PI * THETACORRECTION;
		this->encLeft = msg->encoderLeft;
		this->encRight = msg->encoderRight;
		this->clock = ros::Time::now().toSec();
	}
	return;
}

//Turn to specific absolute angle with PD-Control via odometer
void DiffDriving::turnAbs(double targetTheta)
{
	ros::spinOnce();
	double err = targetTheta - theta;
	if (err > 180) {
		err -= 180;
		this->theta += 360;
	} else if (err < -180) {
		err += 180;
		this->theta -= 360;
	}
	err = abs(err);
	double speed;
	double lasterr = err;
	double dt;
	double derr;
	double derivative = 0;
	double lastspeeds = 0;
	//ROS_INFO("turning to heading %lf", targetTheta);
	while (err > ANGLETOLERANCE) {
		dt = ros::Time::now().toSec() - this->clock;
		ros::spinOnce();
		err = abs(targetTheta - theta);
		if (err != lasterr) {
			lastspeeds = 0;
			derr = err - lasterr;
			derivative = (derr/dt) * KDTURN;
		} else {
			lastspeeds++;
		}
		if (lastspeeds > 100) {
			break;
		}
		ROS_INFO("err: %lf | derr/dt: %lf | err*KPTURN: %lf  Derivative: %lf", err, derr/dt, err*KPTURN, derivative);
		lasterr = err;
		speed = min(MAXTURNSPEED, err * KPTURN + derivative + MINSPEED);
		//if (speed < 0.1) { break; }
		if (theta > targetTheta) {
			this->pdsrv.request.left = -speed;
			this->pdsrv.request.right = speed;
		} else {
			this->pdsrv.request.left = speed;
			this->pdsrv.request.right = -speed;
		}
		diffDrive.call(pdsrv);
	}
	this->stop();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
	return;
}


//Turn specific relative angles with PD-Control via odometer
void DiffDriving::turn(double targetTheta)
{
	ros::spinOnce();
	double goal = theta + targetTheta;
	double err = abs(targetTheta);
	double speed = 1.0;
	double lasterr = err;
	double dt;
	double derr;
	double derivative = 0;
	int lastspeeds = 0;
	double starttime = ros::Time::now().toSec();
	while (err > ANGLETOLERANCE) {
		//ROS_INFO("turning with err: %lf", err);
		dt = ros::Time::now().toSec() - this->clock;
		ros::spinOnce();
		err = abs(goal - theta);
		if (err != lasterr) {
			lastspeeds = 0;
			derr = err - lasterr;
			derivative = (derr/dt) * KDTURN;
		} else {
			lastspeeds++;
		}
		if (lastspeeds > 100) {
			break;
		}
		ROS_INFO("err: %lf | derr/dt: %lf | err*KPTURN: %lf  Derivative: %lf", err, derr/dt, err*KPTURN, derivative);
		lasterr = err;
		//ROS_INFO("Differential component: %lf with derr: %lf and dt: %lf\n lasterr: %lf | err: %lf", KD * (derr/dt), derr, dt, lasterr, err);
		speed =  min(MAXTURNSPEED, err * KPTURN + derivative + MINSPEED);
		//if (speed < 0.1) { break; }
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
	this->stop();
	ROS_INFO("\nxcord: %lf\nycord: %lf\ntheta: %lf\nencLeft: %lf\nencRight: %lf", xcord, ycord, theta, encLeft, encRight);
	return;
}

//PD-control func driving straight for certain distances
void DiffDriving::moveDistEnc(double dist)
{
	ros::spinOnce();
	double speedL;
	double speedR;
	double setpointL = dist*ENCPM + this->encLeft;
	double setpointR = dist*ENCPM + this->encRight;
	double errL = setpointL - this->encLeft;
	double errR = setpointR - this->encRight;
	double lasterrL = errL;
	double lasterrR = errR;
	double dt;
	double derrL;
	double derrR;
	double derivativeL = 0;
	double derivativeR = 0;
	while (abs(errL) > ENCTOLERANCE) {
		if (stop_check()) {
			break;
		}
		dt = ros::Time::now().toSec() - this->clock;
		ros::spinOnce();
		errL = setpointL - this->encLeft;
		//errR = setpointR - this->encRight;
		if (errL != lasterrL) {
			derrL = abs(errL) - abs(lasterrL);
			derivativeL = (derrL/dt) * KD;
		}
		/*
		if (errR != lasterrR) {
			derrR = abs(errR) - abs(lasterrR);
			derivativeR = (derrR/dt) * KD;
		}
		*/
		//ROS_INFO("errL: %lf | errR: %lf | derrL: %lf | derrR: %lf",errL,errR,derrL,derrR);
		speedL = errL * KP;
		//speedR = errR * KP;
		if (errL < 0) {
			speedL -= (MINSPEED + derivativeL);
		} else {
			speedL += (MINSPEED + derivativeL);
		}
		/*
		if (errR < 0) {
			speedR -= (MINSPEED + derivativeR);
		} else {
			speedR += (MINSPEED + derivativeR);
		}
		*/
		pdsrv.request.left = speedL;
		pdsrv.request.right = speedL;
		lasterrL = errL;
		//lasterrR = errR;
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
			if (stay) { return 1; }
			this->turnAbs(targetTheta);
			return 1;
		}
		case 1: {
			double speed;
			turn(calcAngle(x, y));
			double err = calcRelDist(x, y);
			this->moveDistEnc(err);
			if (stay) { return 1; }
			this->turnAbs(targetTheta);
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

void DiffDriving::correct_odometry_location_error_globally(int cell_row, int cell_col){
	double x_dest = cell_col * 0.8;
	double y_dest = cell_row * -0.8;

	centersrv.request.conservative = 1;
	if (CalculateCenterClient.call(centersrv)){
		Eigen::Vector2d rel_center;
		Eigen::Matrix2d rotate_cc;
		double theta_rad = theta * M_PI/180;
		rotate_cc << cos(theta_rad), -sin(theta_rad),
						 sin(theta_rad), cos(theta_rad);
		rel_center(X) = centersrv.response.x;
		rel_center(Y) = centersrv.response.y;
		rel_center = rotate_cc * rel_center;
		double x_pre_correction = this->xcord;
		double y_pre_correction = this->ycord;
		correct_odometry_location_error(x_dest,y_dest,rel_center(X),rel_center(Y));
		ROS_INFO("Corrected odometry location (x,y): (%lf,%lf) -> (%lf,%lf)\n", x_pre_correction, y_pre_correction, this->xcord, this->ycord);
	} else {
		ROS_INFO("CalculateCenterService call failed.");
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

void DiffDriving::correct_odometry_angle_error_globally(int orientation){
	const red_fundamentals::WallsCombined::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::WallsCombined>("walls_combined_topic", ros::Duration(1));
	if(msg != nullptr) {
		Eigen::Vector2d dirvec;
		Eigen::Matrix2d rotate_cc_ninety;
		rotate_cc_ninety << 0, -1,
						 	1, 0;
		int wall_dir = -1;
		double theta_pre_correction = this->theta;
		if (msg->walls[UP]){
			dirvec(X) = msg->F_B[X]-msg->F_A[X];
			dirvec(Y) = msg->F_B[Y]-msg->F_A[Y];
			wall_dir = UP;
		} else if (msg->walls[LEFT]) {
			dirvec(X) = msg->L_B[X]-msg->L_A[X];
			dirvec(Y) = msg->L_B[Y]-msg->L_A[Y];
			wall_dir = LEFT;
		} else if (msg->walls[RIGHT]) {
			dirvec(X) = msg->R_B[X]-msg->R_A[X];
			dirvec(Y) = msg->R_B[Y]-msg->R_A[Y];
			wall_dir = RIGHT;
		} else{
			return;
		}

		dirvec.normalize();
		dirvec = rotate_cc_ninety * dirvec;
		double theta = -1 * calcAngle(dirvec[X],dirvec[Y]); // We have to inver the sign because we enter the wall directional vector and compare it to the robots Y axis so when the robot is +10° calcAngle will output -10° and vice versa
		// ROS_INFO("Theta calcAngle in correction: %lf", theta);

		// adjust angle depending on which wall was used
		switch(wall_dir) {
			case LEFT:
				theta -= 90;
				break;
			case RIGHT: 
				theta += 90;
				break;
			default:
				break;
		}

		// adjust angle depending on which wall was used
		switch(orientation) {
			case LEFT:
				theta -= 90;
				break;
			case RIGHT: 
				theta += 90;
				break;
			case DOWN:
				theta += 180;
				break;
			default:
				break;
		}

		this->theta = theta;
		ROS_INFO("Corrected theta: %lf -> %lf", theta_pre_correction, theta);

	} else {
		ROS_INFO("Could not correct angle globally, wall info not found!");
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
	
	for (int i = 1; i <= 4; i++) {
		driver.turnAbs(driver.theta + 90);
		//driver.turn(90);
		//driver.moveDistEnc(0.25);
		ros::Duration(0.3).sleep();
		//driver.moveDistEnc(0.4);
	}
	ros::Duration(5.0).sleep();
	for (int i = 1; i <= 4; i++) {
		//driver.turnAbs(90*i);
		driver.turn(+90);
		//driver.moveDistEnc(0.25);
		ros::Duration(0.3).sleep();
		//driver.moveDistEnc(0.4);
	}
	
	/*
	driver.drive(0,0,-0.8,0,1);
	driver.drive(0,-0.8,-0.8,0,1);
	driver.drive(0,-0.8,0,0,1);
	driver.drive(0,-1.6,0,0,1);
	driver.drive(0,-1.6,-0.8,0,1);
	driver.drive(0,-1.6,-1.6,0,1);
	driver.drive(0,-2.4,-1.6,0,1);
	driver.drive(0,-2.4,-0.8,0,1);
	driver.drive(0,-2.4,0,0,1);
	driver.drive(0,-3.2,0,0,1);
	driver.drive(0,-4,0,0,1);
	*/
	/*
	driver.drive(0,0.8,0,0,1);
	driver.drive(0,0.8,-0.8,0,1);
	driver.drive(0,0,-0.8,0,1);
	driver.drive(0,0,-1.6,0,1);
	driver.drive(0,0.8,-1.6,0,1);
	driver.drive(0,1.6,-1.6,0,1);
	driver.drive(0,-1.6,-2.4,0,1);
	driver.drive(0,-0.8,-2.4,0,1);
	driver.drive(0,0,-2.4,0,1);
	driver.drive(0,0,-3.2,0,1);
	driver.drive(0,0,-4.0,0,1);
	*/
	/*
	driver.drive(0,-0.8,0,0,1);
	driver.drive(0,-1.6,0,0,1);
	driver.drive(0,-1.6,0.8,0,1);
	driver.drive(0,-2.4,0.8,0,1);
	driver.drive(0,-3.2,0.8,0,1);
	driver.drive(0,-3.2,0,0,1);
	driver.drive(0,-3.2,-0.8,0,1);
	driver.drive(0,-4.0,-0.8,0,1);
	*/

	/*
	driver.turnAbs(180);
	driver.moveDistEnc(0.8);
	driver.turnAbs(-90);
	driver.moveDistEnc(0.8);
	driver.turnAbs(0);
	driver.moveDistEnc(0.8);
	driver.turnAbs(-90);
	driver.moveDistEnc(0.8);
	driver.turnAbs(180);
	driver.moveDistEnc(0.8);
	driver.turnAbs(180);
	driver.moveDistEnc(0.8);
	driver.turnAbs(-90);
	driver.moveDistEnc(0.8);
	driver.turnAbs(0);
	driver.moveDistEnc(0.8);
	driver.turnAbs(0);
	driver.moveDistEnc(0.8);
	driver.turnAbs(-90);
	driver.moveDistEnc(0.8);
	driver.turnAbs(-90);
	driver.moveDistEnc(0.8);
	*/
	//driver.moveDistEnc(4.0);

	return 0;
}
