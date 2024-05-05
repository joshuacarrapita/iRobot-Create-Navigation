#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/Walls.h"
#include <cstdlib>

#define LARRSIZE 726
#define LARROFFSET 44
#define LARRACTSIZE 682
#define ANGLESTEP 0.351906
#define LARRUSED 512
#define LARRUSEDOFFSET 85
#define ANGLESTEPUSED 0.351563
#define TOTALOFFSET 129

using namespace std;

red_fundamentals::Walls wall;

typedef struct Coords
{
	double x;	//for polar = r
	double y;	//for polar = angle
} coords;

const coords robotpos = {.x = 0, .y = 0};


//converts polar to cart coordinates
coords polToCart(coords polar)
{
	//ROS_INFO("Converting polar coordinate with r = %f and angle = %f", polar.x, polar.y);
	coords cart;
	cart.x = polar.x * cos(polar.y);
	cart.y = polar.x * sin(polar.y);
	//ROS_INFO("To cartesian coordinate with (%f , %f)", cart.x, cart.y);
	return cart;
}

//calculates angle of distance measurement in radians
double calcAngle(int index)
{
	return((index * ANGLESTEPUSED/180)* ((double) M_PI));
}

//calculates distance between point and line given through 2 points
double calcDist(coords pt_a, coords pt_b, coords pt_0)
{
	return (abs((pt_b.x-pt_a.x)*(pt_0.y-pt_a.y)-(pt_0.x-pt_a.x)*(pt_b.y-pt_a.y))/sqrt((pt_b.x-pt_a.x)*(pt_b.x-pt_a.x) + (pt_b.y-pt_a.y)*(pt_b.y-pt_a.y)));
}

//calculates slope and y-axis intercept point of line given 2 points
coords calcLineEquation(coords pt_a, coords pt_b)
{
	double slope = (pt_b.y - pt_a.y)/(pt_b.x-pt_a.x);
	double y_inter = slope * pt_a.x - pt_a.y;
	return {.x = slope, .y = y_inter};
}

//simplified RANSAC making use of environment constraints
void sransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	vector<coords> coordinates;
	coords pol;
	//int j = 0;
	//get all valid measurements and convert to cartesian
	for (int i = (TOTALOFFSET-1); i < LARRSIZE - (LARRUSEDOFFSET+1); i++) {
		if (isnan(msg->ranges[i])) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcAngle(i - (TOTALOFFSET-1));
		coordinates.push_back(polToCart(pol));
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", j, coordinates[j].x, coordinates[j].y);
		//j++;
	}
	//actual SRANSAC start
	int maxInliers = 0;
	coords winner_a;
	coords winner_b;
	//take candidates with fixed interval instead of random
	for (int i = 0; i < coordinates.size()-10; i++) {
		int curInliers = 0;
		coords candidate_a = coordinates[i];
		coords candidate_b = coordinates[i+10];
		//count number of inliers
		for (int j = 0; j < coordinates.size(); j++) {
			if (calcDist(candidate_a, candidate_b, coordinates[j]) < 0.05) {
				curInliers++;
			}
		}
		//update winner if current candidates have more inliers than previous best
		if (curInliers > maxInliers) {
			maxInliers = curInliers;
			winner_a = candidate_a;
			winner_b = candidate_b;
			ROS_INFO("winner found with %i inliers", curInliers);
		}
	}
	wall.dist = calcDist(winner_a, winner_b, robotpos);
	//wall.angle = 
	ROS_INFO("Dist to best wall: %.7lf with %i inliers", wall.dist, maxInliers);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "walls");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, sransac);
	ros::Publisher pub = n.advertise<red_fundamentals::Walls>("walls_topic", 1);

	while(ros::ok()) {
		ros::spinOnce();
		//pub.publish(wall);
		ros::Duration(3).sleep();
	}
	return 0;
}
