#include <cstdlib>
#include <cmath>
#include "ros/ros.h"
#include "red_fundamentals/ExecutePlan.h"
#include "create_fundamentals/DiffDrive.h"
#include "sensor_msgs/LaserScan.h"
#include "moving.hpp"

using namespace std;

const int laser_msg_offset = 44;
const double wait_wall_msg = 10;
double safe_wall_dist = 0.8;

enum drive_direction { LEFT = -1, UP = 0, RIGHT = 1, DOWN = 2 };
enum turn_direction { RIGHT_TURN = 1, ALSO_RIGHT_TURN = -3, NO_TURN = 0, LEFT_TURN = -1, ALSO_LEFT_TURN = 3, BACK_TURN = 2, ALSO_BACK_TURN = -2 };
drive_direction robot_dir = UP;

bool execute_plan_callback(red_fundamentals::ExecutePlan::Request &req,
          red_fundamentals::ExecutePlan::Response &res){

	vector<int> exec_plan = req.plan;
	res.success = false;
	if (exec_plan[0] == 1) return true; // maybe this needs to be false?

	Moving mover;

	for (int i = 0; i < exec_plan.size(); i++){

		int next_action = exec_plan[i];
		int turn_dir = robot_dir - next_action;

		switch(turn_dir){

			case LEFT_TURN:
			case ALSO_LEFT_TURN:
				// do left 90° turn
				mover.turnRA(0);
				break;
			
			case RIGHT_TURN:
			case ALSO_RIGHT_TURN:
				// do right 90° turn
				mover.turnRA(1);
				break;
			
			case BACK_TURN:
			case ALSO_BACK_TURN:
				// do right 180° turn
				mover.turnAround();

			default:
				 break;
		
		}
		
		sensor_msgs::LaserScan::ConstPtr laser_message = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan_filtered", ros::Duration(wait_wall_msg));

		if (!laser_message) return false; // maybe this needs to be true still?

		double minDist = 1;

		int valid_ranges_size = laser_message->ranges.size()-laser_msg_offset;

		for (int i = valid_ranges_size/3+laser_msg_offset; i < (valid_ranges_size/3)*2+laser_msg_offset; i++) {
			if (laser_message->ranges[i] < minDist && laser_message->ranges[i] > laser_message->range_min) {
				minDist = laser_message->ranges[i];
			}
		}

		if (minDist < safe_wall_dist) return true; // maybe this needs to be false?

		mover.moveDist(0.8);
	}

	res.success = true;
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_plan_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan_callback);

  ROS_INFO("Ready to run execute plan");

  ros::spin();
  
  return 0;
}
