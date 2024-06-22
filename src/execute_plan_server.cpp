#include <cstdlib>
#include <cmath>
#include "ros/ros.h"
#include "red_fundamentals/ExecutePlan.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "paradriver.hpp"
#include "smooth_plan.hpp"
#include "red_fundamentals/Pose.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentals/WallsCombined.h"
#include "song.hpp"

using namespace std;

enum map_axis { ROW = 0, COL = 1 };

ros::ServiceClient requestloc;
ros::Publisher pose_pub;
red_fundamentals::Pose pose;
red_fundamentals::Grid map_cur;

bool walls[3] = {false,};
vector<int> wall_dirs;
int current_row, current_column, current_orientation;

void mapCallback() {
	while(ros::ok()){
		const red_fundamentals::Grid::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::Grid>("map", ros::Duration(1.0));
		if(msg != nullptr){
			map_cur = *msg;
			ROS_INFO("Received map");
			break;
		}
		else{
			ROS_INFO("Waiting for the map information");
		}
	}
}

void wallCallback(){
	while(ros::ok()) {
		const red_fundamentals::WallsCombined::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::WallsCombined>("walls_combined_topic", ros::Duration(0.2));
		if(msg != nullptr) {
			wall_dirs.clear();
			for (int i = 0; i < 3; ++i) {
				walls[i] = msg->walls[i];
				if (msg->walls[i]) {
					wall_dirs.push_back(i);
				}
			}
			break;
		}

		else{
			ROS_INFO("Waiting for the wall information...");
		}
	}
}

bool compareWalls(red_fundamentals::Cell cell, int orientation){
	if(wall_dirs.size() != cell.walls.size()) return false;
	bool walls_robot[4] = {false,};

	for(int wall : cell.walls){
		// we convert wall direction in the map to one the robot is facing
		switch (orientation) {
			case 0:
				walls_robot[(wall + 1) % 4] = true;
				break;
			case 1:
				walls_robot[wall] = true;
				break;
			case 2:
				walls_robot[(wall + 3) % 4] = true;
				break;
			case 3:
				walls_robot[(wall + 2) % 4] = true;
				break;
			default:
				break;
		}
	}

	for(int i=0; i<3; i++){
		if(walls[i] != walls_robot[i]) return false;
	}
	return true;
}

int interpolate_orientation(vector<int> destination_cell){
	int row = destination_cell[ROW] - current_row;
	int col = destination_cell[COL] - current_column;
	if (row > 0) return 3;
	if (row < 0) return 1;
	if (col > 0) return 0;
	if (col < 0) return 2;
}

void set_pose(vector<int> destination_cell){
	current_orientation = interpolate_orientation(destination_cell);
	current_row = destination_cell[ROW];
	current_column = destination_cell[COL];
	ROS_INFO("current position:");
	ROS_INFO("current_row: %d, current_column:%d, current_orientation:%d", current_row, current_column, current_orientation);
}

vector<vector<int>> exec_plan_to_cell_plan(vector<int> exec_plan, int row, int col){
	vector<vector<int>> cell_plan;
	vector<int> start = {row,col};

	for (int i = 0; i < exec_plan.size(); i++){

		switch (exec_plan[i])
		{
			case RIGHT:
				while(exec_plan[i] == 0){
					start[COL] += 1;
					i++;
				}
				i--;
				break;
			case UP:
				while(exec_plan[i] == 1){
					start[ROW] -= 1;
					i++;
				}
				i--;
				break;
			case LEFT:
				while(exec_plan[i] == 2){
					start[COL] -= 1;
					i++;
				}
				i--;
				break;
			case DOWN:
				while(exec_plan[i] == 3){
					start[ROW] += 1;
					i++;
				}
				i--;
				break;
			default:
				return {{}};
		}

		cell_plan.push_back(start);

	}

	return cell_plan;
}

//set odometry according to current pose
void set_driver_global_coords(DiffDriving driver){
	switch (current_orientation)
	{
		case RIGHT:
			driver.theta = 90;
			break;
		case UP:
			driver.theta = 0;
			break;
		case LEFT:
			driver.theta = -90;
			break;
		case DOWN:
			driver.theta = 180;
			break;
		default:
			break;
	}
	driver.xcord = current_column * 0.8;
	driver.ycord = current_row * (-0.8);
	ROS_INFO("ORIENTATION: %d, DRIVER THETA: %lf,\n POSE (row,col): (%d,%d),\n COORD (x,y): (%lf,%lf)\n",current_orientation, driver.theta, current_row, current_column, driver.xcord, driver.ycord);
}

void correct_odometry(DiffDriving driver){
	//TODO: correct_angle() based on global position and walls instead of center calculation;
	//TODO: correct_coordinates() based on center calculation;
}

bool lost_check(Song s){
	wallCallback();	
	if(!compareWalls(map_cur.rows[current_row].cells[current_column], current_orientation)){
		s.playLostSound();
		ROS_INFO("Lost! We have to relocalize ourselves...");
		return true;
	}
	return false;
}

void publish_pose(){
	pose.row = current_row;
	pose.column = current_column;
	pose.orientation = current_orientation;
	pose_pub.publish(pose);
	ROS_INFO("Pose message published");
}

bool execute_plan(red_fundamentals::ExecutePlan::Request &req, red_fundamentals::ExecutePlan::Response &res)
{
	// initialize variables 
	current_row = req.row;
	current_column = req.column;
	current_orientation = req.orientation;
	res.success = false;
	DiffDriving driver;
	Song s;
	set_driver_global_coords(driver);
	vector<vector<int>> cell_plan = exec_plan_to_cell_plan(req.plan, current_row, current_column); 


	if(!is_executable(current_row, current_column, req.plan, map_cur)) {
		ROS_INFO("Plan not executable! Ready to run another one!");
		return true;
	}

	for(size_t i = 0; i < cell_plan.size(); i++){

		correct_odometry(driver);

		if (lost_check(s)) return true;

		//drive
		double global_x = cell_plan[i][COL] * 0.8;
		double global_y = cell_plan[i][ROW] * -0.8;
		int success = driver.drive(0, global_x, global_y, 0, 1);
		if (!success) {
			s.playLostSound();
			ROS_INFO("Lost! We have to relocalize ourselves...");
			return true;
		}

		set_pose(cell_plan[i]); 
		
		publish_pose();
	}

	res.success = true;
	ROS_INFO("Executed plan! Ready to run another one!");
	ROS_INFO("current pose:  current_row: %d, current_column: %d, current_orientation: %d", current_row, current_column, current_orientation);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "execute_plan_server");
	ros::NodeHandle n;
	pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);

	mapCallback();

	ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan);
	ROS_INFO("Ready to execute plan");
	ros::spin();
	return 0;
}
