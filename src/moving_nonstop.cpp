#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving_nonstop.hpp"
#include <cstdlib>

#define DRIVESPEED (2 * M_PI)
#define TASPEED M_PI
#define WHEELRADIUS 0.033
#define WHEELDISTANCE 0.26


Moving::Moving(){
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
}

void Moving::stop(){
	ddsrv.request.left = 0;
	ddsrv.request.right = 0;
	diffDrive.call(ddsrv);
	return;
}

// TODO: add wall/obstacle detection + stop part

void Moving::move(double dist){
	ddsrv.request.left = DRIVESPEED;
 	ddsrv.request.right = DRIVESPEED;
  	diffDrive.call(ddsrv);
  	ros::Duration(dist/(DRIVESPEED * WHEELRADIUS)).sleep();
}

void Moving::curve(int direction, double angle, double radius){
	// if right curve (direction == 0)
  	double goal_l = fabs((M_PI * angle * (radius + WHEELDISTANCE/2)) / 180);
  	double goal_r = fabs((M_PI * angle * (radius - WHEELDISTANCE/2)) / 180);
  
  	// set speed (first arbitrary)
  	// goal_l : goal_r = speed_l : speed_r
  	double speed_l = TASPEED;
  	double speed_r = speed_l * goal_r / goal_l;
    
  	// else swap
  	if(direction > 0){
   		std::swap(goal_l, goal_r);
   		std::swap(speed_l, speed_r);
  	}
  
  	ddsrv.request.left = speed_l;
  	ddsrv.request.right = speed_r;
  	diffDrive.call(ddsrv);
  	ros::Duration(goal_l /(speed_l * WHEELRADIUS)).sleep();
}
