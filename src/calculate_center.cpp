#include <cstdlib>
#include "ros/ros.h"
#include "red_fundamentals/NewWalls.h"
#include "red_fundamentals/CalculateCenter.h"

using namespace std;

enum CoordinateAxis { X = 0, Y = 1, Z = 2};
enum Point { A = 0, B = 1, C = 2};
enum WallDirection { LEFT = 0, FRONT = 1, RIGHT = 2, BACK = 3};
enum OperationMode { SIMPLE = 0, ADVANCED = 1 };
OperationMode op_mode = ADVANCED;

red_fundamentals::CalculateCenter res_center_coord;
	
const double cell_width = 0.4;
double wall_points[3][2][2];
bool exists_wall[3];
int amt_walls = 0;
bool new_arrival = false;


double* calculate_wall_center_simple(int dir) {

	double A_[2] = {wall_points[dir][A][X],wall_points[dir][A][Y]};
	double B_[2] = {wall_points[dir][B][X],wall_points[dir][B][Y]};
	ROS_INFO("Calculating for A = (%lf,%lf), B = (%lf,%lf)\n", A_[X],A_[Y],B_[X],B_[Y]);
	double BA[2] = {A_[X]-B_[X],A_[Y]-B_[Y]};
	double BA_len = sqrt(pow(BA[X],2) + pow(BA[Y],2));
	double BA_scaled[2] = {BA[X]*cell_width/BA_len,BA[Y]*cell_width/BA_len};
	double BA_rt_ctr_cw[2] = {-1*BA_scaled[Y],BA_scaled[X]};
	double BA_mid[2] = {B_[X]+BA[X]/2,B_[Y]+BA[Y]/2};
	double* wall_center_coord = new double[2];
	wall_center_coord[X] = BA_mid[X]+BA_rt_ctr_cw[X];
	wall_center_coord[Y] = BA_mid[Y]+BA_rt_ctr_cw[Y];
	ROS_INFO("Result = (%lf,%lf)\n", wall_center_coord[X],wall_center_coord[Y]);
	return wall_center_coord;

}

double** calculate_diagonal(double left_wall_points[2][2],double front_wall_points[2][2]){
	double** r = new double*[2];
	return r;
}

double* calculate_intersection(double** left_line,double** right_line){
	double* r = new double[2];
	return r;
}

void calculate_center_coord() {

	switch (op_mode) {
		
		case SIMPLE: {

			double avg_center_coord[2] = {0,0}; // the avg over all walls' center coords

			ROS_INFO("L_A: (%lf,%lf)\n", wall_points[LEFT][A][X], wall_points[LEFT][A][Y]);
			ROS_INFO("L_B: (%lf,%lf)\n", wall_points[LEFT][B][X], wall_points[LEFT][B][Y]);
			ROS_INFO("F_A: (%lf,%lf)\n", wall_points[FRONT][A][X], wall_points[FRONT][A][Y]);
			ROS_INFO("F_B: (%lf,%lf)\n", wall_points[FRONT][B][X], wall_points[FRONT][B][Y]);
			ROS_INFO("R_A: (%lf,%lf)\n", wall_points[RIGHT][A][X], wall_points[RIGHT][A][Y]);
			ROS_INFO("R_B: (%lf,%lf)\n", wall_points[RIGHT][B][X], wall_points[RIGHT][B][Y]);
			// go over all wall directions and calculate the center coordinate for each wall in each direction if a wall was found there
			for (int dir = LEFT; dir < BACK; dir++){
				if (exists_wall[dir]) {
					double* wall_center_coord = calculate_wall_center_simple(dir);
					// sum up the axis of all center wall coords step by step to calculate average later
					avg_center_coord[X] += wall_center_coord[X]; 
					avg_center_coord[Y] += wall_center_coord[Y]; 
				}
			}

			// if we had more than one wall we need to average
			if (amt_walls > 1) {
					for (int axis = X; axis < Z; axis++) {
					avg_center_coord[axis] /= amt_walls;
				}
			}

			res_center_coord.x = avg_center_coord[X];
			res_center_coord.y = avg_center_coord[Y];

			break;

		}

		case ADVANCED: {

			// TODO Implement advanced mode
			switch (amt_walls) {

				case 1: {

					op_mode = SIMPLE;
					calculate_center_coord();
					op_mode = ADVANCED;
					break;

				}

				case 2: {

					if (exists_wall[LEFT] && exists_wall[RIGHT]) {
						
						// TODO: calculate both centers like 1 and average them
						op_mode = SIMPLE;
						calculate_center_coord();
						op_mode = ADVANCED;
						break;


					} else if (exists_wall[LEFT] && exists_wall[FRONT]){

						// TODO: calculation for LEFT and FRONT wall using intersection

						double** diagonal = calculate_diagonal(wall_points[LEFT],wall_points[FRONT]);
						res_center_coord.x = diagonal[0][X] + 0.5 * diagonal[1][X];
						res_center_coord.y = diagonal[0][Y] + 0.5 * diagonal[1][Y];
						

					} else if (exists_wall[FRONT] && exists_wall[RIGHT]){

						// TODO: calculation for FRONT and RIGHT wall using intersection and middle
						double** diagonal = calculate_diagonal(wall_points[FRONT],wall_points[RIGHT]);
						res_center_coord.x = diagonal[0][X] + 0.5 * diagonal[1][X];
						res_center_coord.y = diagonal[0][Y] + 0.5 * diagonal[1][Y];

					}
				}

				case 3: {

					// TODO: calculation for all 3 walls using intersection and intersection
					double** d_L_F = calculate_diagonal(wall_points[LEFT],wall_points[FRONT]);
					double** d_F_R = calculate_diagonal(wall_points[FRONT],wall_points[RIGHT]);
					double* center = calculate_intersection(d_L_F,d_F_R);

					res_center_coord.x = center[X];
					res_center_coord.y = center[Y];

				}

				default: {

					break;

				}

			}

		}
			
		default: {
		
			break;

		}

	}

}

void parse_wall_message(const red_fundamentals::NewWalls::ConstPtr& msg) {

	// Just parses the wall message content into an array for further processing
	amt_walls = 0;
	for (int dir = LEFT; dir < BACK; dir++) exists_wall[dir] = true;

	if((wall_points[LEFT][A][X] = msg->L_A[X]) == 0) exists_wall[LEFT] = false;
	if((wall_points[LEFT][A][Y] = msg->L_A[Y]) == 0) exists_wall[LEFT] = false;
	if((wall_points[LEFT][B][X] = msg->L_B[X]) == 0) exists_wall[LEFT] = false;
	if((wall_points[LEFT][B][Y] = msg->L_B[Y]) == 0) exists_wall[LEFT] = false;
	if((wall_points[FRONT][A][X] = msg->F_A[X]) == 0) exists_wall[FRONT] = false;
	if((wall_points[FRONT][A][Y] = msg->F_A[Y]) == 0) exists_wall[FRONT] = false;
	if((wall_points[FRONT][B][X] = msg->F_B[X]) == 0) exists_wall[FRONT] = false;
	if((wall_points[FRONT][B][Y] = msg->F_B[Y]) == 0) exists_wall[FRONT] = false;
	if((wall_points[RIGHT][A][X] = msg->R_A[X]) == 0) exists_wall[RIGHT] = false;
	if((wall_points[RIGHT][A][Y] = msg->R_A[Y]) == 0) exists_wall[RIGHT] = false;
	if((wall_points[RIGHT][B][X] = msg->R_B[X]) == 0) exists_wall[RIGHT] = false;
	if((wall_points[RIGHT][B][Y] = msg->R_B[Y]) == 0) exists_wall[RIGHT] = false;

	for (int dir = LEFT; dir < BACK; dir++) if (exists_wall[dir]) amt_walls++;

	// ROS_INFO("AMT_WALLS: %d", amt_walls);

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
