#include <cstdlib>
#include "ros/ros.h"
#include "red_fundamentals/Walls.h"

using namespace std;

enum CoordinateAxis { X = 0, Y = 1, Z = 2}
enum Point { A = 0, B = 1, C = 2}
enum WallDirection { LEFT = 0, FRONT = 1, RIGHT = 2, BACK = 3}
enum OperationMode { SIMPLE = 0, ADVANCED = 1 }
OperationMode OpMode = SIMPLE;

red_fundamentals::CalculateCenter res_center_coord;
	
const double cell_width = 0.4;
double wall_points[3][2][2];
bool exists_wall[3];

void parse_wall_message(const red_fundamentals::CalculateCenter::ConstPtr& msg) {

	// Just parses the wall message content into an array for further processing
	exists_wall = {true,true,true};

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

}

double* calculate_wall_center_simple(WallDirection dir) {

	double A[2] = wall_points[dir][0];
	double B[2] = wall_points[dir][1];
	double AB[2] = {A[0]-B[0],A[1]-B[1]}
	double AB_len = sqrt(AB[0]^2 + AB[1]^2)
	double AB_scaled[2] = {AB[0]*cell_width/AB_len,AB[1]*cell_width/AB_len}
	double AB_rt_ctr_cw[2] = {-1*AB[1],AB[0]};
	double AB_mid[2] = {B[0]+AB[0]/2,B[1]+AB[1]/2};
	double wall_center_coord[2] = {AB_mid[0]+AB_rt_ctr_cw[0],AB_mid[1]+AB_rt_ctr_cw[1]};
	return wall_center_coord;

}



void calculate_center_coord() {

	switch (OpMode) {
		
		case SIMPLE:

			double walls_center_coords[3][2]; // arr to save the center coords for each wall
			double avg_center_coord[2] = {0,0}; // the avg over all walls' center coords
			int amt_walls = 0; // the amount of walls we found center coords for

			// go over all wall directions and calculate the center coordinate for each wall in each direction if a wall was found there
			for (WallDirection dir = LEFT; dir < BACK; dir++){
				if (exists_wall[dir]) {
					walls_center_coords[dir] = calculate_wall_center_simple(dir);
					amt_walls++; // increase to calculate average later
					for (CoordinateAxis axis = X, axis < Z, axis++){
						avg_center_coord[axis] += walls_center_coords[dir][axis]; // sum up the axis of all center wall coords step by step to calculate average later
					}
				}
			}

			// if we had more than one wall we need to average
			if (amt_walls > 0) {
				for (CorrdinateAxis axis = X, axis < Z, axis++){
					avg_center_coord[axis] /= amt_walls;
				}
			}

			res_center_coord.x = avg_center_coord[0];
			res_center_coord.y = avg_center_coord[1];

			break;

		case ADVANCED:
			// TODO Implement advanced mode
			break;
			
	}

}

void parse_args(int argc, char**argv){

	if (argc != 2 || argv[1] != "SIMPLE" || argv[1] != "ADVANCED") {
		ROS_ERROR("Usage calculate_center <OperationMode {\"SIMPLE\", "\ADVANCED\"}>");
		ROS_INFO("Setting default values...");
	} else if (argv[1] == "SIMPLE") {
		OpMode = SIMPLE;
	} else {
		OpMode = ADVANCED;
	}

}

int main(int argc, char **argv){

	// set mode of Operation
	parse_args();

	ros::init(argc, argv, "calculate_center");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("walls_topic", 1, parse_wall_message);
	ros::Publisher pub = n.advertise<red_fundamentals::CalculateCenter>("calculate_center_topic", 1);

	while(ros::ok()) {
		ros::spinOnce(); // parse wall information
		calculate_center_coord(); // updates res_center_coord with new center
		pub.publish(res_center_coord); // TODO maybe make a choice if a coordinate should be published based on if the new calculation is more or less "robust" than the previous one, maybe because less walls were used
	}

	return 0;

}
