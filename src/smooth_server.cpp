#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/ExecutePlan.h"
#include "red_fundamentals/StopMsg.h"
#include "create_fundamentals/ResetEncoders.h"
#include "smooth_plan.hpp"
#include "moving.hpp"
#include "map.hpp"

using namespace std;

#define DRIVESPEED (2 * M_PI)
#define CURVESPEED (1.5 * M_PI)
#define WHEELRADIUS 0.033
#define WHEELDISTANCE 0.26

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive ddsrv;
ros::ServiceClient resetEncoders; 
ros::Publisher pose_pub;
red_fundamentals::Pose pose;
int row, column, orientation;
double x, y, theta;

const double Kp = 1;

double l_delta;
double r_delta;

double l_act;
double r_act;

double error_l;
double error_r;

double speed_l;
double speed_r;

const double tolerance = 1;


void get_current_pose(){
  // get a message from pose once
  while(ros::ok()){
  	const red_fundamentals::Pose::ConstPtr& msg = ros::topic::waitForMessage<red_fundamentals::Pose>("/pose", ros::Duration(1.0));
  
  	if (msg != nullptr){
    		row = msg->row;
    		column = msg->column;
    		orientation = msg->orientation;
    		y = 0.4 + 0.8 * row;
    		x = 0.4 + 0.8 * column;
    		theta = -M_PI/2 + (M_PI/2) * orientation;
    		ROS_INFO("Pose message arrived");
    		ROS_INFO("Pose after localization");
    		ROS_INFO("row: %d, column: %d, orientation: %d", row, column, orientation);
		ROS_INFO("x: %f, y: %f, theta: %f", x, y, theta);
    		break;
  	}
  	
  	else{
    		ROS_INFO("Waiting for the pose message from localization node");
  	}
  }
}

bool stop_check(){

	red_fundamentals::StopMsg::ConstPtr stop_message = ros::topic::waitForMessage<red_fundamentals::StopMsg>("stop_topic", ros::Duration(1));

	if (stop_message && stop_message->content == "STOP") return true;
	return false;

}

void publish_pose(){
	pose.row = row;
	pose.column = column;
	pose.orientation = orientation;
	pose_pub.publish(pose);
}

void sensorCallback2(){

	create_fundamentals::SensorPacket::ConstPtr msg = ros::topic::waitForMessage<create_fundamentals::SensorPacket>("sensor_packet", ros::Duration(10));
	if(!msg){
		return;
	}
	l_delta = msg->encoderLeft - l_act;
	r_delta = msg->encoderRight - r_act;
	l_act = msg->encoderLeft;
	r_act = msg->encoderRight;
	double dist_d = WHEELRADIUS * (l_delta + r_delta) / 2;
	double theta_d = (r_delta - l_delta) / WHEELDISTANCE;
	ROS_INFO("dist difference: %f, theta difference: %f", dist_d, theta_d);
	x += dist_d * cos(theta + theta_d/2);
	y += dist_d * sin(theta + theta_d/2);
	theta += theta_d;
	ROS_INFO("x: %f, y: %f, theta: %f", x, y, theta);
	row = floor(y/0.8);
	column = floor(x/0.8);
	orientation = (int)round((theta / (M_PI / 2))/90) + 1;
	publish_pose();
}

bool pd_control(double dist)
{

	double speed;
	double errL = dist*32;
	double errR = dist*32;
	double errT = errL+errR/2;
	// ROS_INFO("Moving %lf m",dist);
	sensorCallback2();
	double goalL = errL + l_act;
	double goalR = errR + r_act;
	// ROS_INFO("Callback called: left: %f, right: %f", l_act, r_act);
	while (errT > tolerance) {
		if (stop_check()) {
			ddsrv.request.left = 0;
			ddsrv.request.right = 0;
			diffDrive.call(ddsrv);
			return false;
		}
		sensorCallback2();
		errT = (goalL-l_act)+(goalR-r_act)/2;
		speed = abs(min(10.0, (errT * Kp + 0.4)));
		ddsrv.request.left = speed;
		ddsrv.request.right = speed;
		diffDrive.call(ddsrv);
	}

	// ROS_INFO("Callback called: left: %f, right: %f", l_act, r_act);
	return true;

}

bool sleep_and_stop(double duration){

	ros::Time then = ros::Time::now() + ros::Duration(duration);
	while (ros::Time::now() <= then) {
		if (stop_check()) {
			ddsrv.request.left = 0;
			ddsrv.request.right = 0;
			diffDrive.call(ddsrv);
			return false;
		}
		sensorCallback2();
	}
	return true;

}

bool curve(int direction, double angle, double radius){

	// if right curve (direction == 0)
	double goal_l = M_PI * angle * (radius + WHEELDISTANCE/2) / 180;
	double goal_r = M_PI * angle * (radius - WHEELDISTANCE/2) / 180;

	// set speed (first arbitrary)
	// goal_l : goal_r = speed_l : speed_r
	double speed_l = CURVESPEED;
	double speed_r = speed_l * goal_r / goal_l;

	// else swap
	if(direction > 0){
		std::swap(goal_l, goal_r);
		std::swap(speed_l, speed_r);
	}

	ddsrv.request.left = speed_l;
	ddsrv.request.right = speed_r;
	diffDrive.call(ddsrv);
	return sleep_and_stop(fabs(goal_l /(speed_l * WHEELRADIUS))*0.85);

}

void resetOdometer()
{

	create_fundamentals::ResetEncoders srv;
	resetEncoders.call(srv);
	// ROS_INFO("encoders reset");
	
}

bool move_direction(int movement, double goal, Moving mover){

	switch(movement){
		// for curve 0: right curve 1: left curve
		case RIGHT_CURVE:
			return curve(0, goal, 0.4);

		case GO_STRAIGHT:
			return pd_control(goal);

		case LEFT_CURVE:
			return curve(1, goal, 0.4);

		case ROTATE:
			switch((int) goal){

				case -90:
					// do left 90° turn
					mover.turnRA(0);
					break;

				case 90:
					// do right 90° turn
					mover.turnRA(1);
					break;

				case 180:
					// do right 180° turn
					mover.turnAround();
					break;

				default:
					break;

			}
			break;
	}
	return true;

}

bool execute_plan_smooth(red_fundamentals::ExecutePlan::Request  &req,
		red_fundamentals::ExecutePlan::Response &res)
{
	Moving mover;

	// initialize variables 
	res.success = false;

	// check if the plan is executable
  	// TODO get map outside of this function if it is possible
  	Map map = Map();
  	if(!is_executable(row, column, req.plan, &map)) return true;
  
	// plan the movements
	vector<pair<int, double>> movements = compute_plan(get_direction(req.plan), orientation);


	// simplify the plan if needed
	merge_plan(movements);
	resetOdometer();
	for(auto movement : movements){
		if(!move_direction(movement.first, movement.second, mover)){
			resetOdometer();
			ROS_INFO("Could not execute smooth plan! Ready to run another one.");
			return true;
		}
	}

	mover.stop(); // This is so it doesn't keep driving after the very last
	//p_control drive instruction
	
	res.success = true;
	resetOdometer();
	ROS_INFO("Executed smooth plan! Ready to run another one!");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "smooth_server");
	ros::NodeHandle n;
	
	// TODO call it everytime when we relocalize
  	get_current_pose();
  	
  	pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);
	ros::ServiceServer service = n.advertiseService("execute_smooth_plan", execute_plan_smooth);
	resetEncoders = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

	ROS_INFO("Ready to run smooth plan");
	ros::spin();

	return 0;
}

