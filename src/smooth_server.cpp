#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/ExecutePlan.h"
#include "moving_nonstop.hpp"
#include "smooth_plan.hpp"

using namespace std;

Moving mover;
  
/* void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
} */

bool move_direction(int movement, double goal){
  switch(movement){
  	// for curve 0: right curve 1: left curve
  	case RIGHT_CURVE:
  		mover.curve(0, goal, 0.4);
  		break;
  		
  	case GO_STRAIGHT:
  		mover.move(goal);
  		break;
  		
  	case LEFT_CURVE:
  		mover.curve(1, goal, 0.4);
  		break;
  	
  	case ROTATE:
		// cw
  		if(goal > 0) mover.curve(0, goal, 0);
  		// ccw
  		else mover.curve(1, fabs(goal), 0);
  		break;
  }
}

bool execute_plan_smooth(red_fundamentals::ExecutePlan::Request  &req,
          red_fundamentals::ExecutePlan::Response &res)
{

  // initialize variables 
  res.success = false;
  
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
  	if(!move_direction(movement.first, movement.second)){
  		return true;
  	}
  }
  
  mover.stop();
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smooth_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan_smooth);
  // ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
  
  ROS_INFO("Ready to run smooth plan");
  ros::spin();
  
  return 0;
}
