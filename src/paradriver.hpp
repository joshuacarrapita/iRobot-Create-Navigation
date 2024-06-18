#ifndef PARADRIVER_H
#define PARADRIVER_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

enum dir {dr = 0, du = 1, dl = 2, dd = 3};

class DiffDriving
{
	public:
		void stop();
		void resetOdometer();
		int odomInit();
		double calcAngle(double x, double y);
		double calcDist(double x, double y);
		double encoderToDist(double encDiff);
		double centDist(double distLeft, double distRight);
		double thetaDiff(double distLeft, double distRight);
		void odometer(const create_fundamentals::SensorPacket::ConstPtr& msg);
		void turnAbs(double targetTheta);
		void turn(double targetTheta);
		void smoothTurn(double x, double y, double targetTheta);
		void moveDist(double dist, dir direction);
		void moveDistEnc(double dist);
		int drive(int mode, double x, double y, double targetTheta, int stay);
		void testdrive(double leftspeed, double rightspeed);
		void moveDist(double dist);
		void correct_odometry_location_error(double x_dest, double y_dest, double x_center_rel_act, double y_center_rel_act);
		void correct_odometry_angle_error(double x_center_rel_act, double y_center_rel_act);
		ros::NodeHandle n;
		ros::ServiceClient diffDrive;
		create_fundamentals::DiffDrive pdsrv;
		ros::Subscriber pdsub;
		int odomReady;
		double encLeft;
		double encRight;
		double xcord;
		double ycord;
		double theta;
		double clock;
		DiffDriving();
};

#endif
