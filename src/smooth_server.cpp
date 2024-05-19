#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/ExecutePlan.h"
#include "red_fundamentals/StopMsg.h"
#include "smooth_plan.hpp"
#include "moving.hpp"

using namespace std;

#define DRIVESPEED (2 * M_PI)
#define CURVESPEED (1.5 * M_PI)
#define WHEELRADIUS 0.0305
#define WHEELDISTANCE 0.26

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive ddsrv;

const double Kp = 1.1;

double l_act;
double r_act;

double error_l;
double error_r;

double speed_l;
double speed_r;

const double tolerance = 0.25;

bool calling = false;
bool kill_driver = false;

bool stop_check(){
	red_fundamentals::StopMsg::ConstPtr stop_message = ros::topic::waitForMessage<red_fundamentals::StopMsg>("stop_topic", ros::Duration(1));

	if (stop_message && stop_message->content == "STOP") return true;
	return false;
}

void sensorCallback2()
{
  create_fundamentals::SensorPacket::ConstPtr msg = ros::topic::waitForMessage<create_fundamentals::SensorPacket>("sensor_packet", ros::Duration(0.2));
  if(!msg){
    return;
  }
  l_act = msg->encoderLeft;
  r_act = msg->encoderRight;
  ROS_INFO("Callback called: left: %f, right: %f", l_act, r_act);
  calling = true;

}

// TODO: replace moving functions

void pd_control(double dist){
  ROS_INFO("Start of pd control");
  
  // float dist = 0.8 * streak;
  ROS_INFO("We want to go %f m straight", dist);

  while(!calling){
    ROS_INFO("No encoder values yet");
    sensorCallback2();
  }
  
  double l_des = l_act + (dist * 32);
  double r_des = r_act + (dist * 32);

  ROS_INFO("Actual Encoder Values: left: %f, right: %f", l_act, r_act);
  ROS_INFO("Desired Encoder Values: left: %f, right: %f", l_des, r_des);

  while(ros::ok()){

	if (stop_check()) {
      	ddsrv.request.left = 0;
      	ddsrv.request.right = 0;
      	diffDrive.call(ddsrv);
		break;
	}
  
    sensorCallback2();

    ROS_INFO("Actual Encoder Values: left: %f, right: %f", l_act, r_act);

    ROS_INFO("Desired Encoder Values: left: %f, right: %f", l_des, r_des);

    error_l = l_des - l_act;
    error_r = r_des - r_act;

    ROS_INFO("Error: left: %f, right: %f", error_l, error_r);

    if(l_act + tolerance >= l_des || r_act + tolerance >= r_des){       // Goal is reached
      
      break;

    } else if(r_act != 0 || l_act != 0){        // p-Control: calculating speed
      
      speed_l = Kp * error_l;
      speed_r = Kp * error_r;
      ddsrv.request.left = speed_l;
      ddsrv.request.right = speed_r;
      diffDrive.call(ddsrv);

    } else {                                    // no encoder values yet
      
      ROS_INFO("No encoder values yet");
    }

  }

  ROS_INFO("End of pd control! Stop");

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
  
  if (stop_check()) {
	  mover.stop();
	  ROS_INFO("stopping");
	  return false;
  }
  
  switch(movement){
  	// for curve 0: right curve 1: left curve
  	case RIGHT_CURVE:
  		curve(0, goal, 0.4);
  		break;
  		
  	case GO_STRAIGHT:
  		pd_control(goal);
  		break;
  		
  	case LEFT_CURVE:
  		curve(1, goal, 0.4);
  		break;
  	
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
  
  // after centering, the robot will always face the wall
  // immediate failure, if it wants to go up
  if(req.plan[0] == 1){
  	return true;
  }
  
  // plan the movements
  vector<pair<int, double>> movements = compute_plan((get_direction(req.plan)));

  
  // simplify the plan if needed
  merge_plan(movements);

  for(auto movement : movements){
  	if(!move_direction(movement.first, movement.second, mover)){
		if(kill_driver) {
			system("bash -c 'source ~/red_ws/devel/setup.bash ; rosnode kill /create'");
			system("bash -c 'source ~/red_ws/devel/setup.bash ; rosrun create_fundamentals driver.py'");
		}
  		return true;
  	}
  }
  
  mover.stop();
  if(kill_driver) {
	system("bash -c 'source ~/red_ws/devel/setup.bash ; rosnode kill /create'");
	system("bash -c 'source ~/red_ws/devel/setup.bash ; rosrun create_fundamentals driver.py'");
  }
  res.success = true;
  ROS_INFO("Ready to run smooth plan");
  return true;
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		ROS_ERROR("Usage smooth_server <bool kill_driver{0,1}>");
		ROS_INFO("Setting default values...");
	} else {
		kill_driver = (bool) atoi(argv[1]);
	}

  ros::init(argc, argv, "smooth_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan_smooth);
  diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  ROS_INFO("Ready to run smooth plan");
  ros::spin();
  
  return 0;
}
