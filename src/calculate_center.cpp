#include <cstdlib>
#include "ros/ros.h"
#include "red_fundamentals/WallsCombined.h"
#include "red_fundamentals/CalculateCenter.h"
#include <Eigen/Dense>

using namespace std;

#define NORMALIZE true
#define OFFSET 0.15

enum CoordinateAxis { X = 0, Y = 1, Z = 2};
enum Point { A = 0, B = 1, C = 2};
enum WallDirection { LEFT = 0, FRONT = 1, RIGHT = 2, BACK = 3};
enum OperationMode { SIMPLE = 0, ADVANCED = 1 };
enum ParamForm { START = 0, DIRVEC = 1 };
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
	//ROS_INFO("Calculating for A = (%lf,%lf), B = (%lf,%lf)\n", A_[X],A_[Y],B_[X],B_[Y]);
	double BA[2] = {A_[X]-B_[X],A_[Y]-B_[Y]};
	double BA_len = sqrt(pow(BA[X],2) + pow(BA[Y],2));
	double BA_scaled[2] = {BA[X]*cell_width/BA_len,BA[Y]*cell_width/BA_len};
	double BA_rt_ctr_cw[2] = {-1*BA_scaled[Y],BA_scaled[X]};
	double BA_mid[2] = {B_[X]+BA[X]/2,B_[Y]+BA[Y]/2};
	double* wall_center_coord = new double[2]; // DONE: Pointer memory deletion checked
	wall_center_coord[X] = BA_mid[X]+BA_rt_ctr_cw[X];
	wall_center_coord[Y] = BA_mid[Y]+BA_rt_ctr_cw[Y];
	//ROS_INFO("Result = (%lf,%lf)\n", wall_center_coord[X],wall_center_coord[Y]);
	return wall_center_coord;

}

double** get_line_param_form(double given_wall_points[2][2], bool normalize = false) {

	double** r = new double*[2]; // DONE: Pointer memory deletion checked
	for (int i = 0; i < 3; i++) r[i] = new double[2]; // DONE: Pointer memory deletion checked


	r[START][X] = given_wall_points[A][X];
	r[START][Y] = given_wall_points[A][Y];
	r[DIRVEC][X] = given_wall_points[B][X]-given_wall_points[A][X];
	r[DIRVEC][Y] = given_wall_points[B][Y]-given_wall_points[A][Y];

	if (normalize) {
		Eigen::Vector2d dirvec;
		dirvec(X) = r[DIRVEC][X];
		dirvec(Y) = r[DIRVEC][Y];
		dirvec.normalize();
		r[DIRVEC][X] = dirvec(X);
		r[DIRVEC][Y] = dirvec(Y);
	}

	return r;

}

double* get_line_point(double** param_form_line, double lambda) {

	double* r = new double[2]; // DONE: Pointer memory deletion checked
	r[X] = param_form_line[START][X] + lambda * param_form_line[DIRVEC][X];
	r[Y] = param_form_line[START][Y] + lambda * param_form_line[DIRVEC][Y];
	return r;

}

double* calculate_intersection(double** left_line, double** right_line){

	Eigen::Matrix2d A;
	A(0,0) = left_line[DIRVEC][X];
	A(1,0) = left_line[DIRVEC][Y];
	A(0,1) = -1*right_line[DIRVEC][X];
	A(1,1) = -1*right_line[DIRVEC][Y];

	Eigen::Vector2d b;
	b(X) = right_line[START][X] - left_line[START][X];
	b(Y) = right_line[START][Y] - left_line[START][Y];

	Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

	double* r = get_line_point(left_line, x(0));
	return r;

}

double** calculate_diagonal(double left_wall_points[2][2], double right_wall_points[2][2]){

	double l_w_p[2][2];
	copy(&left_wall_points[0][0], &left_wall_points[0][0]+2*2, &l_w_p[0][0]);
	double r_w_p[2][2];
	copy(&right_wall_points[0][0], &right_wall_points[0][0]+2*2, &r_w_p[0][0]);

	double** left_wall_model = get_line_param_form(l_w_p);
	double** right_wall_model = get_line_param_form(r_w_p);
	//ROS_INFO("LEFT WALL ParamForm | StartPoint = (%lf,%lf), DirectionalVector = (%lf,%lf)\n", left_wall_model[START][X], left_wall_model[START][Y], left_wall_model[DIRVEC][X], left_wall_model[DIRVEC][Y]);
	//ROS_INFO("RIGHT WALL ParamForm | StartPoint = (%lf,%lf), DirectionalVector = (%lf,%lf)\n", right_wall_model[START][X], right_wall_model[START][Y], right_wall_model[DIRVEC][X], right_wall_model[DIRVEC][Y]);

	// find new intersection point and recalculate the line models
	double* intersection = calculate_intersection(left_wall_model, right_wall_model);
	//ROS_INFO("INTERSECTION = (%lf,%lf)\n", intersection[X], intersection[Y]);

	// Set new start vector for left and right walls and change direction of model
	l_w_p[B][X] = l_w_p[A][X];
	l_w_p[B][Y] = l_w_p[A][Y];
	l_w_p[A][X] = intersection[X];
	l_w_p[A][Y] = intersection[Y];
	r_w_p[A][X] = intersection[X];
	r_w_p[A][Y] = intersection[Y];
	for (int i = 0; i < 3; i++) {
		delete left_wall_model[i];
		delete right_wall_model[i];
	}
	delete intersection;
	delete left_wall_model;
	delete right_wall_model;
	left_wall_model = get_line_param_form(l_w_p, NORMALIZE);
	right_wall_model = get_line_param_form(r_w_p, NORMALIZE);
	//ROS_INFO("LEFT WALL ParamForm | StartPoint = (%lf,%lf), DirectionalVector = (%lf,%lf)\n", left_wall_model[START][X], left_wall_model[START][Y], left_wall_model[DIRVEC][X], left_wall_model[DIRVEC][Y]);
	//ROS_INFO("RIGHT WALL ParamForm | StartPoint = (%lf,%lf), DirectionalVector = (%lf,%lf)\n", right_wall_model[START][X], right_wall_model[START][Y], right_wall_model[DIRVEC][X], right_wall_model[DIRVEC][Y]);

	double* d_A = get_line_point(left_wall_model, 0.8);
	double* d_B = get_line_point(right_wall_model, 0.8);
	for (int i = 0; i < 3; i++) {
		delete left_wall_model[i];
		delete right_wall_model[i];
	}
	delete left_wall_model;
	delete right_wall_model;
	double diagonal_points[2][2];
	diagonal_points[A][X] = d_A[X];
	diagonal_points[A][Y] = d_A[Y];
	diagonal_points[B][X] = d_B[X];
	diagonal_points[B][Y] = d_B[Y];
	delete d_A;
	delete d_B;
	//ROS_INFO("DIAGONAL POINTS: A = (%lf,%lf), B = (%lf,%lf)\n", diagonal_points[A][X], diagonal_points[A][Y], diagonal_points[B][X], diagonal_points[B][Y]);
	
	double** diagonal_line_param_form = get_line_param_form(diagonal_points);
	return diagonal_line_param_form;

}

void calculate_center_coord() {

	switch (op_mode) {
		
		case SIMPLE: {

			double avg_center_coord[2] = {0,0}; // the avg over all walls' center coords

			//ROS_INFO("L_A: (%lf,%lf)\n", wall_points[LEFT][A][X], wall_points[LEFT][A][Y]);
			//ROS_INFO("L_B: (%lf,%lf)\n", wall_points[LEFT][B][X], wall_points[LEFT][B][Y]);
			//ROS_INFO("F_A: (%lf,%lf)\n", wall_points[FRONT][A][X], wall_points[FRONT][A][Y]);
			//ROS_INFO("F_B: (%lf,%lf)\n", wall_points[FRONT][B][X], wall_points[FRONT][B][Y]);
			//ROS_INFO("R_A: (%lf,%lf)\n", wall_points[RIGHT][A][X], wall_points[RIGHT][A][Y]);
			//ROS_INFO("R_B: (%lf,%lf)\n", wall_points[RIGHT][B][X], wall_points[RIGHT][B][Y]);
			// go over all wall directions and calculate the center coordinate for each wall in each direction if a wall was found there
			
			double* wall_center_coord;
			for (int dir = LEFT; dir < BACK; dir++){
				if (exists_wall[dir]) {
					wall_center_coord = calculate_wall_center_simple(dir);
					// sum up the axis of all center wall coords step by step to calculate average later
					avg_center_coord[X] += wall_center_coord[X]; 
					avg_center_coord[Y] += wall_center_coord[Y]; 
				}
			}
			delete wall_center_coord;	

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

			// DONE Implement advanced mode
			switch (amt_walls) {

				case 1: {

					op_mode = SIMPLE;
					calculate_center_coord();
					op_mode = ADVANCED;

					if (exists_wall[LEFT] || exists_wall[RIGHT]) {
						res_center_coord.y = 0; // set y to 0 because we can only adjust with x since y is not realiably calculated
					} else if (exists_wall[FRONT]) {
						res_center_coord.x = 0; // set y to 0 because we can only adjust with x since y is not realiably calculated
					}


					break;

				}

				case 2: {

					if (exists_wall[LEFT] && exists_wall[RIGHT]) {
						
						// DONE: calculate both centers like 1 and average them
						op_mode = SIMPLE;
						calculate_center_coord();
						op_mode = ADVANCED;

						res_center_coord.y = 0; // set y to 0 because we can only adjust with x since y is not realiably calculated

						break;


					} else if (exists_wall[LEFT] && exists_wall[FRONT]){

						// DONE: calculation for LEFT and FRONT wall using intersection

						double** diagonal = calculate_diagonal(wall_points[LEFT],wall_points[FRONT]);
						//ROS_INFO("DIAGONAL | StartPoint = (%lf,%lf), DirectionalVector = (%lf,%lf)\n", diagonal[START][X], diagonal[START][Y], diagonal[DIRVEC][X], diagonal[DIRVEC][Y]);

						double* center = get_line_point(diagonal, 0.5);
						res_center_coord.x = center[X];
						res_center_coord.y = center[Y];
						for (int i = 0; i < 3; i++) delete diagonal[i];
						delete diagonal;
						delete center;

					} else if (exists_wall[FRONT] && exists_wall[RIGHT]){

						// DONE: calculation for FRONT and RIGHT wall using intersection and middle
						double** diagonal = calculate_diagonal(wall_points[FRONT],wall_points[RIGHT]);
						double* center = get_line_point(diagonal, 0.5);
						res_center_coord.x = center[X];
						res_center_coord.y = center[Y];
						for (int i = 0; i < 3; i++) delete diagonal[i];
						delete diagonal;
						delete center;
						
					}

					break;

				}

				case 3: {

					// DONE: calculation for all 3 walls using intersection and intersection
					//ROS_INFO("AMT_POINTS: %d", amt_walls);
					//ROS_INFO("Calculating for L_A = (%lf,%lf), L_B = (%lf,%lf)\n F_A = (%lf,%lf), F_B = (%lf,%lf)\n, R_A = (%lf,%lf), R_B = (%lf,%lf)\n", wall_points[LEFT][A][X], wall_points[LEFT][A][Y], wall_points[LEFT][B][X], wall_points[LEFT][B][Y], wall_points[FRONT][A][X], wall_points[FRONT][A][Y], wall_points[FRONT][B][X], wall_points[FRONT][B][Y], wall_points[RIGHT][A][X], wall_points[RIGHT][A][Y], wall_points[RIGHT][B][X], wall_points[RIGHT][B][Y]);
					//ROS_INFO("Calculating L_F diagonal for 3 walls.");
					double** d_L_F = calculate_diagonal(wall_points[LEFT],wall_points[FRONT]);
					//ROS_INFO("Calculating F_R diagonal for 3 walls.");
					double** d_F_R = calculate_diagonal(wall_points[FRONT],wall_points[RIGHT]);
					//ROS_INFO("Calculating diagonal intersection for 3 walls.");
					double* center = calculate_intersection(d_L_F,d_F_R);
					//ROS_INFO("Setting result center for 3 walls.");
					//ROS_INFO("Result = (%lf,%lf)\n", center[X],center[Y]);
					res_center_coord.x = center[X];
					res_center_coord.y = center[Y];
					for (int i = 0; i < 3; i++) {
						delete d_L_F[i];
						delete d_F_R[i];
					}
					delete d_L_F;
					delete d_F_R;
					delete center;

					break;

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

void parse_wall_message(const red_fundamentals::WallsCombined::ConstPtr& msg) {

	// Just parses the wall message content into an array for further processing
	amt_walls = 0;
	res_center_coord.x = 0;
	res_center_coord.y = 0;

	for (int dir = LEFT; dir < BACK; dir++) exists_wall[dir] = true;

	if((wall_points[LEFT][A][X] = msg->L_A[X]) == 0) exists_wall[LEFT] = false;
	if((wall_points[LEFT][A][Y] = msg->L_A[Y]+OFFSET) == 0) exists_wall[LEFT] = false;
	if((wall_points[LEFT][B][X] = msg->L_B[X]) == 0) exists_wall[LEFT] = false;
	if((wall_points[LEFT][B][Y] = msg->L_B[Y]+OFFSET) == 0) exists_wall[LEFT] = false;
	if((wall_points[FRONT][A][X] = msg->F_A[X]) == 0) exists_wall[FRONT] = false;
	if((wall_points[FRONT][A][Y] = msg->F_A[Y]+OFFSET) == 0) exists_wall[FRONT] = false;
	if((wall_points[FRONT][B][X] = msg->F_B[X]) == 0) exists_wall[FRONT] = false;
	if((wall_points[FRONT][B][Y] = msg->F_B[Y]+OFFSET) == 0) exists_wall[FRONT] = false;
	if((wall_points[RIGHT][A][X] = msg->R_A[X]) == 0) exists_wall[RIGHT] = false;
	if((wall_points[RIGHT][A][Y] = msg->R_A[Y]+OFFSET) == 0) exists_wall[RIGHT] = false;
	if((wall_points[RIGHT][B][X] = msg->R_B[X]) == 0) exists_wall[RIGHT] = false;
	if((wall_points[RIGHT][B][Y] = msg->R_B[Y]+OFFSET) == 0) exists_wall[RIGHT] = false;

	for (int dir = LEFT; dir < BACK; dir++) if (exists_wall[dir]) amt_walls++;

	//ROS_INFO("AMT_WALLS: %d", amt_walls);
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
	ros::Subscriber sub = n.subscribe("walls_combined_topic", 1, parse_wall_message);
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
