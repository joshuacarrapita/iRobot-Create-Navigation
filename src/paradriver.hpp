#ifndef PARADRIVER_H
#define PARADRIVER_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "red_fundamentals/Light.h"
#include "red_fundamentals/CalculateCenterService.h"
#include "red_fundamentals/WallsCombined.h"

enum dir {dr = 0, du = 1, dl = 2, dd = 3};

class DiffDriving
{
	public:
		void stop();
		void resetOdometer();
		int odomInit();
		void stop_check(const red_fundamentals::Light::ConstPtr& stopmsg);
		double calcAngle(double x, double y);
		double calcDist(double x, double y);
		double calcRelDist(double x, double y);
		double encoderToDist(double encDiff);
		double centDist(double distLeft, double distRight);
		double thetaDiff(double distLeft, double distRight);
		void odometer(const create_fundamentals::SensorPacket::ConstPtr& msg);
		void turnAbs(double targetTheta);
		void turn(double targetTheta);
		int  moveDistEnc(double dist);
		int drive(int mode, double x, double y, double targetTheta, int stay);

		void correct_odometry_angle_error_globally(int orientation);
		void correct_odometry_location_error_globally(int cell_row, int cell_col);
		void correct_odometry_location_error(double x_dest, double y_dest, double x_center_rel_act, double y_center_rel_act);
		void correct_odometry_angle_error(double x_dest, double y_dest, double x_center_rel_act, double y_center_rel_act);

		ros::NodeHandle n;
		ros::ServiceClient diffDrive;
		ros::ServiceClient CalculateCenterClient;
		red_fundamentals::CalculateCenterService centersrv;
		create_fundamentals::DiffDrive pdsrv;
		ros::Subscriber pdsub;
		ros::Subscriber stopsub;
		int odomReady;
		double encLeft;
		double encRight;
		bool bumperLeft;
		bool bumperRight;
		bool shouldStop;
		double xcord;
		double ycord;
		double theta;
		double clock;
		DiffDriving();
};

#endif
