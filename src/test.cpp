#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

double speed = M_PI; // pi * r m/s
double r = 0.033;
double d = 0.26;

void stop(){
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);
}

void move(double l, double theta){
	double goal_s = fabs(l);
	if(l < 0) speed *= -1;
	srv.request.left = speed;
	srv.request.right = speed;
	diffDrive.call(srv);
	ros::Duration(goal_s/(M_PI * r)).sleep();
	stop();
	
	double goal_r = fabs((M_PI * d * theta) / 360);
	if(theta < 0) speed *= -1;
	srv.request.left = -speed;
	srv.request.right = speed;
	diffDrive.call(srv);
	ros::Duration(goal_r /(M_PI * r)).sleep();
	stop();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "example");
	ros::NodeHandle n;
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	
	move(-1, -90);  

	ros::spin();
	return 0;
}

