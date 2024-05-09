#include "ros/ros.h"
#include "red_fundamentals/ExecutePlan.h"
#include <cstdlib>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"

using namespace std;

const int laser_msg_offset = 44;
const double wait_wall_msg = 10;
double safe_wall_dist = 0.8;

enum drive_direction { LEFT = -1, UP = 0, RIGHT = 1, DOWN = 2 }
enum turn_direction { RIGHT_TURN = 1, ALSO_RIGHT_TURN = -3, NO_TURN = 0, LEFT_TURN = -1, ALSO_LEFT_TURN = 3, BACK_TURN = 2, ALSO_BACK_TURN = -2 }
drive_direction robot_dir = UP;


bool execute_plan_callback(red_fundamentals::ExecutePlan::Request &exec_plan,
          red_fundamentals::ExecutePlan::Response &res){

	res.success = false;
	if (exec_plan[0] == 1) return true; // maybe this needs to be false?

	for (int i = 0; i < exec_plan.size(); i++){

		int next_action = exec_plan[i];
		int turn_dir = robot_dir - next_action;

		switch(turn_dir){

			case (LEFT_TURN || ALSO_LEFT_TURN):
				// do left 90° turn
			
			case (RIGHT_TURN || ALSO_RIGHT_TURN):
				// do left 90° turn
			
			case (BACK_TURN || ALSO_BACK_TURN):
				// do left 90° turn
		
		}
		
		sensor_msgs::LaserScan::ConstPtr& laser_message = ros::topic::waitForMessage<sensor_msgs:LaserScan>("scan_filtered", ros::Duration::(wait_wall_msg));

		if (!laser_message) return false; // maybe this needs to be true still?

		double minDist = 1;

		int valid_ranges_size = msg->ranges.size()-laser_msg_offset;

		for (int i = valid_ranges_size/3+laser_msg_offset; i < (valid_ranges_size/3)*2+laser_msg_offset; i++) {
			if (msg->ranges[i] < minDist && msg->ranges[i] > msg->range_min) {
				minDist = msg->ranges[i];
			}
		}

		if (minDist < safe_wall_dist) return true; // maybe this needs to be false?


		// move(0.8);
	}

	res.success = true;
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_plan_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan);
  ROS_INFO("Ready to run execute plan");

  ros::spin();
  
  return 0;
}

