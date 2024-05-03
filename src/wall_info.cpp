#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/WallInfo.h"

using namespace std;
char** args = NULL;

red_fundamentals::WallInfo winfo_msg;
const float angle_step = 0.3515669;
const int laser_msg_offset = 44;
const int laser_msg_size = 682;
const int laser_msg_center = 341;
const float robot_center_offset = 0.15;
const float wall_length = 0.4;
int amt_coordinates = 0;
int amt_inliers = 0;

float tuple_dot_prod(tuple<float,float> x, tuple<float,float> y) {

	return get<0>(x)*get<0>(y)+get<1>(x)*get<1>(y);

}

tuple<float,float> tuple_add(tuple<float,float> x, tuple<float,float> y) {

	return {get<0>(x)+get<0>(y),get<1>(x)+get<1>(y)};

}

tuple<float,float> tuple_sub(tuple<float,float> x, tuple<float,float> y) {

	return {get<0>(x)-get<0>(y),get<1>(x)-get<1>(y)};

}

tuple<float,float> tuple_mul_scalar(tuple<float,float> x, float y) {

	return {get<0>(x)*y,get<1>(x)*y};

}

float tuple_norm(tuple<float,float> x) {

	return sqrt(tuple_dot_prod(x,x));

}

tuple<float,float> project_x_onto_param_form(tuple<float,float> x, vector<tuple<float,float>> model) {

	tuple<float,float> start_vec = model[0];
	tuple<float,float> dir_vec = model[1];

	float lambda = (tuple_dot_prod(x,dir_vec)-
			tuple_dot_prod(start_vec,dir_vec))/
		tuple_dot_prod(dir_vec,dir_vec);

	return tuple_add(start_vec, tuple_mul_scalar(dir_vec,lambda));

}

tuple<float,float> tuple_rotate_counter_clockwise_90(tuple<float,float> x) {

	return {-1*get<1>(x),get<0>(x)};

}

tuple<float,float> tuple_rotate_clockwise_90(tuple<float,float> x) {

	return {get<1>(x),-1*get<0>(x)};

}

tuple<float,float> calc_move_point(vector<tuple<float,float>> model_param_form, bool neg = false) {

	tuple<float,float> start = model_param_form[0];
	tuple<float,float> dir_vec = model_param_form[1];
	tuple<float,float> mid = tuple_add(start,tuple_mul_scalar(dir_vec,0.5));
	tuple<float,float> dir_vec_norm = tuple_mul_scalar(dir_vec,1/tuple_norm(dir_vec));
	tuple<float,float> dir_vec_norm_rot;

	if (neg) {

		dir_vec_norm_rot = tuple_rotate_clockwise_90(dir_vec_norm);

	} else {
		
		dir_vec_norm_rot = tuple_rotate_counter_clockwise_90(dir_vec_norm);

	}

	return {tuple_add(mid, tuple_mul_scalar(dir_vec_norm_rot,sqrt(wall_length)))};
	
}


vector<float> ransac (

		vector<tuple<float,float>> coordinates, // points we use to model the wall
		int poly_deg = 1, // the shape of wall to look for
		int epochs = 300, // the amount of rounds to try finding the best wall
		float margin = 0.05, // for which distance points are still considered part of the wall
		int success = 30, // amt of model-fitting points needed to consider it a wall
		int angle_tolerance = 0 // how much bigger of an angle a better wall may have
		){

	amt_inliers = 0;
	amt_coordinates = coordinates.size();

	vector<tuple<float,float>> best_model_param_form = {{-98,-98},{-98,-98}};

	for (int i = 0; i < epochs; i++){

		// get amt_points unique random inlier candidates from cartesian points
		srand(time(NULL));
		tuple<float,float> inlier_candidates[2];

		int k;
		int l;
		do {

			k = rand()%coordinates.size();
			l = rand()%coordinates.size();

		}
		while (k == l);

		inlier_candidates[0] = coordinates[k];
		inlier_candidates[1] = coordinates[l];

		// model parameters to candidates
		vector<tuple<float,float>> candidate_model;
		tuple<float,float> dir_vec;

 		// make sure the directional vectore goes from right to left
		if (k < l) {

			dir_vec = tuple_sub(inlier_candidates[0],inlier_candidates[1]);
			candidate_model.push_back(inlier_candidates[1]); // index 1 is starting point

		} else {

			dir_vec = tuple_sub(inlier_candidates[1],inlier_candidates[0]);
			candidate_model.push_back(inlier_candidates[0]); // index 0 is starting point

		}
		candidate_model.push_back(dir_vec); // dir_vec is direction vector


		// TODO: Implement matmul and gauss to make it work with any polynomial
		vector<tuple<float,float>> inliers_confirmed;

		for (int i = 0; i < coordinates.size(); i++){

			// find orthogonal projection x_p of x onto model
			tuple<float,float> x = coordinates[i];
			tuple<float,float> x_p = project_x_onto_param_form(x, candidate_model);

			// if distance to x is less than margin add to confirmed inliers
			float x_dist = tuple_norm(tuple_sub(x_p,x));

			if (x_dist < margin) {
				inliers_confirmed.push_back(x);
				//ROS_INFO("Dist of x to model: %f\n", x_dist);
			}

		}

		// update success value and best_model if we have found a good model
		if (inliers_confirmed.size() >= success){

			success, amt_inliers = inliers_confirmed.size();
			best_model_param_form = candidate_model;

		}

	}
		
	if (get<0>(best_model_param_form[0]) != -98){

		tuple<float,float> move_point = calc_move_point(best_model_param_form);
		tuple<float,float> move_point_2 = calc_move_point(best_model_param_form,true);

		if (tuple_norm(move_point_2) < tuple_norm(move_point)) {
			move_point = move_point_2;
		}

		tuple<float,float> dir_vec = best_model_param_form[1];
		tuple<float,float> dir_vec_norm = tuple_mul_scalar(dir_vec,1/tuple_norm(dir_vec));

		return {get<0>(move_point),get<1>(move_point),get<0>(dir_vec_norm),get<1>(dir_vec_norm)};

	} else {
		
		return {-98,-98,-98,-98};

	}

}

// converts polar to cartesian coordinates
tuple<float,float> polarToCartesian(float radius, float angle) {

	float x = sin(angle)*radius;
	float y = cos(angle)*radius;
	return {x,y};

}

void findWall(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {

	// parse arguments for ransac
	unsigned int epochs = (unsigned int) abs(atoi(args[2]));
	float margin = (float) atof(args[3]);
	unsigned int success = (unsigned int) abs(atoi(args[4]));
	unsigned int angle_tolerance = (unsigned int) abs(atoi(args[5]));

	// set up a coordinate array and fill it with cartesian points
	vector<tuple<float,float>> coordinates;

	int j = 0; // count the entries in coordinates array
	for (int i = 0; i < laser_msg_size; i++) {

		float dist = laser_msg->ranges[i+laser_msg_offset]; // read distance from laser messages and skip first unusable entries

		if (dist == dist){ // do not continue if NaN

			int i_angle_offset = i - laser_msg_center; // i = 0 -> angle = -120 deg
			float angle = i_angle_offset*angle_step; // compute angle for laser message

			tuple<float,float> point = polarToCartesian(dist, angle); // get caresian point				
			get<1>(point) += robot_center_offset; // adjust so (0,0) is the center of the robot)

			coordinates.push_back(point); // add it to coordinates vector

			//ROS_INFO("Converted polar (r,a) -> cartesian (x,y): (%f,%f) -> (%f,%f)\n", dist, angle, get<0>(coordinates[j]), get<1>(coordinates[j]));

			j++;

		}

	}

	// find a wall in that array
	vector<float> wall = {-99,-99,-99,-99};
	if (args) {

		wall = ransac(coordinates, 1, epochs, margin, success, angle_tolerance);

	} else {

		wall = ransac(coordinates);

	}

	winfo_msg.x_move = wall[0];
	winfo_msg.y_move = wall[1];
	winfo_msg.x_dir_prll = wall[2];
	winfo_msg.y_dir_prll = wall[3];

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "wall_info_talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<red_fundamentals::WallInfo>("wall_info_topic", 100);
	ros::Subscriber sub = n.subscribe("scan_filtered", 100, findWall);

	if (argc != 6) {

		ROS_INFO("usage: loop_rate epochs margin success angle_tolerance");

	} else {

		args = argv;

	}

	float l_r = (float) atof(argv[1]);

	ros::Rate loop_rate(l_r);
	while (ros::ok()){

		ros::spinOnce();

		pub.publish(winfo_msg);

		ROS_INFO("WallInfo msg: Move Point: (%f,%f), Parallel Dir Vec: (%f,%f), inliers: %d, of total points: %d\n", winfo_msg.x_move, winfo_msg.y_move, winfo_msg.x_dir_prll, winfo_msg.y_dir_prll, amt_inliers, amt_coordinates);

		loop_rate.sleep();

	}

	return 0;

}
