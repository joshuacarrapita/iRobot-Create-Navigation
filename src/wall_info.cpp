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
int amt_coordinates = 0;
int amt_inliers = 0;

float tuple_dot_prod(tuple<float,float> x, tuple<float,float> y){
	return get<0>(x)*get<0>(y)+get<1>(x)*get<1>(y);
}

tuple<float,float> tuple_add(tuple<float,float> x, tuple<float,float> y){
	return {get<0>(x)+get<0>(y),get<1>(x)+get<1>(y)};
}

tuple<float,float> tuple_sub(tuple<float,float> x, tuple<float,float> y){
	return {get<0>(x)-get<0>(y),get<1>(x)-get<1>(y)};
}

tuple<float,float> tuple_mul_scalar(tuple<float,float> x, float y){
	return {get<0>(x)*y,get<1>(x)*y};
}

float tuple_norm(tuple<float,float> x){
	return sqrt(tuple_dot_prod(x,x));
}

tuple<float,float> project_x_onto_param_form(tuple<float,float> x,
		vector<tuple<float,float>> model){
	tuple<float,float> start_vec = model[0];
	tuple<float,float> dir_vec = model[1];

	float lambda = (tuple_dot_prod(x,dir_vec)-
			tuple_dot_prod(start_vec,dir_vec))/
		tuple_dot_prod(dir_vec,dir_vec);

	return tuple_add(start_vec, tuple_mul_scalar(dir_vec,lambda));
}

tuple<float,float> ransac(
		vector<tuple<float,float>> coordinates, // points we use to model the wall
		int poly_deg = 1, // the shape of wall to look for
		int epochs = 200, // the amount of rounds to try finding the best wall
		float margin = 0.2, // for which distance points are still considered part of the wall
		int success = 10, // amt of model-fitting points needed to consider it a wall
		int angle_tolerance = 0 // how much bigger of an angle a better wall may have
		){

	int amt_points = poly_deg+1; // the amount of points needed to model the polynomial
	amt_inliers = 0;
	amt_coordinates = coordinates.size();

	vector<tuple<float,float>> best_model_param_form;
	tuple<float,float> best_model_polar_form = {-2,0};
	float best_angle = 999;

	vector<tuple<float,float>> zero_deg_model = {{0,0},{1,0}}; // TODO: Check if coordinate system makes problems

	for (int i = 0; i < epochs; i++){

		// get amt_points unique random inlier candidates from cartesian points
		srand(time(NULL));
		tuple<float,float> inlier_candidates[amt_points];
		tuple<float,float> u = {0,0};

		while(tuple_norm(u) < 0.5){
			bool used[coordinates.size()] = {0};

			for (int j = 0; j < amt_points; j++){
				int k;
				do k = rand()%coordinates.size();
				while (used[k] == true);

				inlier_candidates[j] = coordinates[k];

				used[k] = true;
			}
			u = tuple_sub(inlier_candidates[1],inlier_candidates[0]);
		}

		// model parameters to candidates
		vector<tuple<float,float>> candidate_model;
		// TODO: Implement matmul and gauss to make it work with any polynomial

		// create a linear model in parameter form
		candidate_model.push_back(inlier_candidates[0]); // index 0 is starting point
		candidate_model.push_back(u); // u is direction vector

		vector<tuple<float,float>> inliers_confirmed;

		for (int i = 0; i < coordinates.size(); i++){

			// find orthogonal projection x_p of x onto model
			tuple<float,float> x = coordinates[i];
			tuple<float,float> x_p = project_x_onto_param_form(x, candidate_model);

			// if distance to x is less than margin add to confirmed inliers
			float x_dist = tuple_norm(tuple_sub(x,x_p));

			if (x_dist < margin) {
				inliers_confirmed.push_back(x);
				//ROS_INFO("Dist of x to model: %f\n", x_dist);
			}

		}

		// update success value and best_model if we have found a good model
		if (inliers_confirmed.size() >= success){
			// get angle and dist of model to laser
			tuple<float,float> x = {0,0};
			tuple<float,float> x_c_model = project_x_onto_param_form(x, candidate_model);
			tuple<float,float> x_0_model = project_x_onto_param_form(x_c_model, zero_deg_model);
			float angle = acos(tuple_norm(x_0_model) / tuple_norm(x_c_model));
			float dist = tuple_norm(x_c_model);

			// only if the angle is also better than before it's a good model
			if (angle <= best_angle+angle_tolerance){
				success = inliers_confirmed.size();
				if (angle <= best_angle) best_angle = angle;
				best_model_param_form = candidate_model;
				best_model_polar_form = {dist,angle};
			}
		}
	}
	amt_inliers = success;
	return best_model_polar_form;
}

// converts polar to cartesian coordinates
tuple<float,float> polarToCartesian(float radius, float angle) {
	float x = cos(angle)*radius;
	float y = sin(angle)*radius;
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

			coordinates.push_back(point); // add it to coordinates vector

			//ROS_INFO("Converted polar (r,a) -> cartesian (x,y): (%f,%f) -> (%f,%f)\n", dist, angle, get<0>(coordinates[j]), get<1>(coordinates[j]));

			j++;
		}
	}

	// find a wall in that array
	tuple<float,float> wall = {-1,0};
	if (args) {
	    wall = ransac(coordinates, 1, epochs, margin, success, angle_tolerance);
	} else {
	    wall = ransac(coordinates);
	}

	winfo_msg.dist = get<0>(wall);
	winfo_msg.angle = get<1>(wall) * (180.0/3.14159265358979323);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wall_info_talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<red_fundamentals::WallInfo>("wall_info_topic", 100);
	ros::Subscriber sub = n.subscribe("scan_filtered", 100, findWall);

	if (argc != 6) {
            ROS_INFO("usage: loop_rate epochs margin success angle_tolerance");
	} else {
	    args = argv;
	}

	float l_r = atof(argv[1]);

	ros::Rate loop_rate(l_r);
	while (ros::ok()){
		ros::spinOnce();

		pub.publish(winfo_msg);
		ROS_INFO("WallInfo msg: dist: %f, angle: %f, inliers: %d, of total points: %d\n", winfo_msg.dist, winfo_msg.angle, amt_inliers, amt_coordinates);

		loop_rate.sleep();
	}

	return 0;
}
