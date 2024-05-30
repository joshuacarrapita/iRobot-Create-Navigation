#include <cstdlib>
#include "ros/ros.h"
#include "red_fundamentals/NewWalls.h"
#include "red_fundamentals/CalculateCenter.h"

using namespace std;

enum CoordinateAxis { X = 0, Y = 1, Z = 2};
enum Point { A = 0, B = 1, C = 2};
enum WallDirection { LEFT = 0, FRONT = 1, RIGHT = 2, BACK = 3};
enum OperationMode { SIMPLE = 0, ADVANCED = 1 };
OperationMode op_mode = SIMPLE;

red_fundamentals::CalculateCenter res_center_coord;
	
const double cell_width = 0.4;
double wall_points[3][2][2];
bool exists_wall[3];
bool new_arrival = false;


double* calculate_wall_center_simple(WallDirection dir) {

	double A[2] = {wall_points[dir][0][0],wall_points[dir][0][1]};
	double B[2] = {wall_points[dir][1][0],wall_points[dir][1][1]};
	ROS_INFO("Calculating for A = (%lf,%lf), B = (%lf,%lf)\n", A[0],A[1],B[0],B[1]);
	double BA[2] = {A[0]-B[0],A[1]-B[1]};
	double BA_len = sqrt(pow(BA[0],2) + pow(BA[1],2));
	double BA_scaled[2] = {BA[0]*cell_width/BA_len,BA[1]*cell_width/BA_len};
	double BA_rt_ctr_cw[2] = {-1*BA_scaled[1],BA_scaled[0]};
	double BA_mid[2] = {B[0]+BA[0]/2,B[1]+BA[1]/2};
	double* wall_center_coord = new double[2];
	wall_center_coord[0] = BA_mid[0]+BA_rt_ctr_cw[0];
	wall_center_coord[1] = BA_mid[1]+BA_rt_ctr_cw[1];
	ROS_INFO("Result = (%lf,%lf)\n", wall_center_coord[0],wall_center_coord[1]);
	return wall_center_coord;

}

void calculate_center_coord() {

	switch (op_mode) {
		
		case SIMPLE: {

			double walls_center_coords[3][2]; // arr to save the center coords for each wall
			double avg_center_coord[2] = {0,0}; // the avg over all walls' center coords
			int amt_walls = 0; // the amount of walls we found center coords for

			ROS_INFO("L_A: (%lf,%lf)\n", wall_points[0][0][0], wall_points[0][0][1]);
			ROS_INFO("L_B: (%lf,%lf)\n", wall_points[0][1][0], wall_points[0][1][1]);
			ROS_INFO("F_A: (%lf,%lf)\n", wall_points[1][0][0], wall_points[1][0][1]);
			ROS_INFO("F_B: (%lf,%lf)\n", wall_points[1][1][0], wall_points[1][1][1]);
			ROS_INFO("R_A: (%lf,%lf)\n", wall_points[2][0][0], wall_points[2][0][1]);
			ROS_INFO("R_B: (%lf,%lf)\n", wall_points[2][1][0], wall_points[2][1][1]);
			// go over all wall directions and calculate the center coordinate for each wall in each direction if a wall was found there
			for (WallDirection dir = LEFT; dir < BACK; dir = WallDirection(dir+1)){
				if (exists_wall[dir]) {
					double* wall_center_coord = calculate_wall_center_simple(dir);
					walls_center_coords[dir][0] = wall_center_coord[0];
					walls_center_coords[dir][1] = wall_center_coord[1];
					amt_walls++; // increase to calculate average later
					for (CoordinateAxis axis = X; axis < Z; axis = CoordinateAxis(axis+1)){
						avg_center_coord[axis] += walls_center_coords[dir][axis]; // sum up the axis of all center wall coords step by step to calculate average later
					}
				}
			}

			// if we had more than one wall we need to average
			if (amt_walls > 1) {
					for (CoordinateAxis axis = X; axis < Z; axis = CoordinateAxis(axis+1)){
					avg_center_coord[axis] /= amt_walls;
				}
			}

			res_center_coord.x = avg_center_coord[0];
			res_center_coord.y = avg_center_coord[1];

			break;
		}

		case ADVANCED: {
			// TODO Implement advanced mode
			break;

		}
			
	}

}

void parse_wall_message(const red_fundamentals::NewWalls::ConstPtr& msg) {

	// Just parses the wall message content into an array for further processing
	for (int i = 0; i < 3; i++){
		exists_wall[i] = true;
	}

	if((wall_points[0][0][0] = msg->L_A[0]) == 0) exists_wall[0] = false;
	if((wall_points[0][0][1] = msg->L_A[1]) == 0) exists_wall[0] = false;
	if((wall_points[0][1][0] = msg->L_B[0]) == 0) exists_wall[0] = false;
	if((wall_points[0][1][1] = msg->L_B[1]) == 0) exists_wall[0] = false;
	if((wall_points[1][0][0] = msg->F_A[0]) == 0) exists_wall[1] = false;
	if((wall_points[1][0][1] = msg->F_A[1]) == 0) exists_wall[1] = false;
	if((wall_points[1][1][0] = msg->F_B[0]) == 0) exists_wall[1] = false;
	if((wall_points[1][1][1] = msg->F_B[1]) == 0) exists_wall[1] = false;
	if((wall_points[2][0][0] = msg->R_A[0]) == 0) exists_wall[2] = false;
	if((wall_points[2][0][1] = msg->R_A[1]) == 0) exists_wall[2] = false;
	if((wall_points[2][1][0] = msg->R_B[0]) == 0) exists_wall[2] = false;
	if((wall_points[2][1][1] = msg->R_B[1]) == 0) exists_wall[2] = false;

	calculate_center_coord();
	new_arrival = true;

}

void parse_args(int argc, char**argv){

	if (argc != 2 || argv[1] != "SIMPLE" || argv[1] != "ADVANCED") {
		ROS_ERROR("Usage calculate_center <OperationMode {\"SIMPLE\", \"ADVANCED\"}>");
		ROS_INFO("Setting default values...");
	} else if (argv[1] == "SIMPLE") {
		op_mode = SIMPLE;
	} else {
		op_mode = ADVANCED;
	}

}

int main(int argc, char **argv){

	// set mode of Operation
	parse_args(argc, argv);

	ros::init(argc, argv, "calculate_center");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("new_walls_topic", 1, parse_wall_message);
	ros::Publisher pub = n.advertise<red_fundamentals::CalculateCenter>("calculate_center_topic", 1);

	while(ros::ok()) {
		ros::spinOnce(); // parse wall information
		if(new_arrival){
			pub.publish(res_center_coord); // TODO maybe make a choice if a coordinate should be published based on if the new calculation is more or less "robust" than the previous one, maybe because less walls were used
			new_arrival = false;
		}
	}

	return 0;

}
