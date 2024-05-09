#include "ros/ros.h"
#include "red_fundamentals/ExecutePlan.h"
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

bool move(double l, double theta){
	// first rotate theta degrees (ccw)
	speed = M_PI;
	double goal_r = fabs((M_PI * d * theta) / 360);
	if(theta < 0) speed *= -1;
	srv.request.left = -speed;
	srv.request.right = speed;
	diffDrive.call(srv);
	ros::Duration(goal_r /(M_PI * r)).sleep();
	stop();
	
	// TODO: check if there is a wall
	
	// then move l m
	speed = M_PI;
	double goal_s = fabs(l);
	if(l < 0) speed *= -1;
	srv.request.left = speed;
	srv.request.right = speed;
	diffDrive.call(srv);
	ros::Duration(goal_s/(M_PI * r)).sleep();
	stop();
	
	return true;
}


// compute the rotation angle and call a function to execute the movement
// reverse the rotation in the previous action + rotation for the current action

bool move_direction(int dir, int dir_prev){ // 0: Right, 1: Up, 2: Left, 3: Down
	
	// if dir == dir_prev theta = 0 (moving in the same direction twice in a row)
	if(dir == dir_prev) return move(80, 0);
	
	double theta, theta_prev;
	theta_prev = -90 + 90 * dir_prev;
	theta = -theta_prev -90 + 90 * dir;
	
	// if |theta| > 180, rotate in the opposite direction to minimize movement
	if(theta < -180) theta = 360 + theta;
	else if(theta > 180) theta = -(360 - theta);
	
	return move(0.8, theta);
}

bool execute_plan(red_fundamentals::ExecutePlan::Request  &req,
          red_fundamentals::ExecutePlan::Response &res)
{
  // after centering, the robot will always face the wall
  // immediate failure, if it wants to go up
  if(req.plan[0] == 1){
  	res.success = false;
  	return true;
  }
  for(size_t i = 0; i < req.plan.size(); i++){
  	// if i=0, the direction the robot is pointing is UP(=1)
  	if(!move_direction(req.plan[i], (i < 1) ? 1 : req.plan[i-1])){
  		res.success = false;
  		return true;;
  	}
  }
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_plan_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan);
  diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  
  ROS_INFO("Ready to run execute plan");
  ros::spin();
  
  return 0;
}

