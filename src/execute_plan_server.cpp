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
const double safe_wall_dist = 0.8;

enum drive_direction { RIGHT = 0, UP = 1, LEFT = 2, DOWN = 3 };
enum turn_direction { RIGHT_TURN = 1, ALSO_RIGHT_TURN = -3, NO_TURN = 0, LEFT_TURN = -1, ALSO_LEFT_TURN = 3, BACK_TURN = 2, ALSO_BACK_TURN = -2 };

double wall_distance_check(){
	sensor_msgs::LaserScan::ConstPtr laser_message = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan_filtered", ros::Duration(wait_wall_msg));

	if (!laser_message) return -1; // return if "scan_filtered" didn't provide a message

 	// adjust for the first unusable values
	int valid_ranges_size = laser_message->ranges.size()-laser_msg_offset;

	/* The wanderer code scanned too wide of a radius 
	 * so I chose a different option.
	 * We could take the average over a few values in the middle if 
	 * we want to be sure we dont hit an outlier but this 
	 * narrow radius is generally more reliable.
	 */
	// get the distance in front of the robot
	double dist = laser_message->ranges[valid_ranges_size/2+laser_msg_offset]; 

	// if it is below the legal distance we can return something that is safe
	if (dist < laser_message->range_min) return safe_wall_dist+0.1; 

	return dist; // otherwise we return the distance for comparison
}

bool execute_plan_callback(red_fundamentals::ExecutePlan::Request &req,
          red_fundamentals::ExecutePlan::Response &res){

	// Initialize and reset variables between callbacks
	vector<int> exec_plan = req.plan;
	res.success = false;
	drive_direction robot_dir = UP;
	Moving mover;
	
	// go over the whole execution plan
	for (int i = 0; i < exec_plan.size(); i++){

		int next_action = exec_plan[i];
		int turn_dir = robot_dir - next_action; // dynamically calculate the necessary turn amount based on current robot direction and desired cell
		robot_dir = (drive_direction) next_action; // update robot direction

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
				break;

			default:
				break;
		
		}
		
		// before moving, check for wall; this also catches the case of trying to move up after centering
		double dist = wall_distance_check();
		if (dist < safe_wall_dist) {
			if (dist < 0) {
				ROS_INFO("I couldn't find any laser message!");
				return false;
			}
			ROS_INFO("Almost hit a wall! Stopping!\n");
			return true;
		}

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
