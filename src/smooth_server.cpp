#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/ExecutePlan.h"
#include "smooth_plan.hpp"
#include "moving.hpp"

using namespace std;

#define DRIVESPEED (2 * M_PI)
#define CURVESPEED (1.5 * M_PI)
#define WHEELRADIUS 0.032
#define WHEELDISTANCE 0.26

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive ddsrv;

  
/* void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
} */

void stop(){
	ddsrv.request.left = 0;
	ddsrv.request.right = 0;
	diffDrive.call(ddsrv);
	return;
}

// TODO: add wall/obstacle detection + stop part

void move(double dist){
	ddsrv.request.left = DRIVESPEED;
 	ddsrv.request.right = DRIVESPEED;
  	diffDrive.call(ddsrv);
  	ros::Duration(dist/(DRIVESPEED * WHEELRADIUS)).sleep();
}

void curve(int direction, double angle, double radius){
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
  	ros::Duration(fabs(goal_l /(speed_l * WHEELRADIUS))).sleep();
}

bool move_direction(int movement, double goal, Moving mover){
// TODO: add wall/obstacle detection part
  switch(movement){
  	// for curve 0: right curve 1: left curve
  	case RIGHT_CURVE:
  		curve(0, goal, 0.4);
  		break;
  		
  	case GO_STRAIGHT:
  		move(goal);
  		break;
  		
  	case LEFT_CURVE:
  		curve(1, goal, 0.4);
  		break;
  	
  	case ROTATE:
		switch((int) goal){
			case -90:
				mover.turnRA(0);
				break;
			case 90:
				mover.turnRA(1);
				break;
			case 180:
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

  // initialize variables 
  res.success = false;
  Moving mover;
  
  // after centering, the robot will always face the wall
  // immediate failure, if it wants to go up
  if(req.plan[0] == 1){
  	return true;
  }
  
  // plan the movements
  vector<pair<int, double>> movements = compute_plan((get_direction(req.plan)));
  
  // simplify the plan if needed
  // merge_plan(movements);
  
  for(auto movement : movements){
  	ROS_INFO("direction: %d, goal: %f", movement.first, movement.second);
  	if(!move_direction(movement.first, movement.second, mover)){
  		return true;
  	}
  }
  
  mover.stop();
  ROS_INFO("moving finished");
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smooth_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan_smooth);
  diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  // ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
  
  ROS_INFO("Ready to run smooth plan");
  ros::spin();
  
  return 0;
}

