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

double errThresh = 0.05;
int candidateDist = 10;

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
double calcSensAngle(int index)
{
	//return((index * ANGLESTEP/240)* ((double) M_PI));
	return((-30.0 + ((index * 240)/682)) * ((double) M_PI /180));
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
	double y_inter = pt_a.y - slope * pt_a.x;
	return {.x = slope, .y = y_inter};
}

//calculates angle relative to y axis via 2 winner points
double calcAngle(coords pt_a, coords pt_b)
{
	//double ydiff = max(pt_a.y,pt_b.y) - min(pt_a.y,pt_b.y);
	double ydiff = pt_a.y-pt_b.y;
	return ((acos(ydiff/sqrt((pt_a.x-pt_b.x)*(pt_a.x-pt_b.x) + ydiff*ydiff))) * (180/(double) M_PI));
}

//calculates angle relative to y axis via slope of line
double calcAngleAlt(double slope)
{
	return (cos(slope)/sin(slope));
}

//simplified RANSAC making use of environment constraints
void extendedSransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	vector<coords> coordinates;
	coords pol;
	//int j = 0;
	//get all valid measurements and convert to cartesian
	for (int i = LARROFFSET; i < LARRSIZE; i++) {
		if (isnan(msg->ranges[i]) && msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates.push_back(polToCart(pol));
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", j, coordinates[j].x, coordinates[j].y);
		//j++;
	}
	//actual SRANSAC start
	vector<coords> inliers;
	vector<coords> outliers;
	vector<coords> winner_outliers;
	int maxInliers = 0;
	coords winner_a;
	coords winner_b;
	//find 3 best walls; remove inliers each iteration
	for (int k = 0; k < 3; k++) {
		//take candidates with fixed interval instead of random
		for (int i = 0; i < coordinates.size()-(candidateDist+1); i+=candidateDist) {
			//ROS_INFO("checking candidates %i", i);
			int curInliers = 0;
			coords candidate_a = coordinates[i];
			coords candidate_b = coordinates[i+candidateDist];
			//count number of inliers
			for (int j = 0; j < coordinates.size(); j++) {
				if (calcDist(candidate_a, candidate_b, coordinates[j]) < errThresh) {
					curInliers++;
					inliers.push_back(coordinates[j]);
				} else {
					outliers.push_back(coordinates[j]);
				}
			}
			//update winner if current candidates have more inliers than previous best
			if (curInliers > maxInliers) {
				
				if (k < 4) {
					for (int i = 0; i < inliers.size(); i++) {
						ROS_INFO("inlier coord: (%lf , %lf)", inliers[i].x, inliers[i].y);
					}
				}
				int outliersize = outliers.size();
				winner_outliers = outliers;
				int woutliersize = winner_outliers.size();
				maxInliers = curInliers;
				ROS_INFO("outliers of size: %i now set as winner_outliers of size: %i | inliers: %i", outliersize, woutliersize, curInliers);
				winner_a = candidate_a;
				winner_b = candidate_b;
			}
			inliers.clear();
			outliers.clear();
		}
		coordinates = winner_outliers;
		int newsize = winner_outliers.size();
		ROS_INFO("new candidate size: %i", newsize);
		wall.dist.push_back(calcDist(winner_a, winner_b, robotpos));
		wall.angle.push_back(calcAngle(winner_a, winner_b));
		ROS_INFO("Best wall %i with %i inliers has dist: %.7lf angle: %.7lf", k, maxInliers, wall.dist[k], wall.angle[k]);
		maxInliers = 0;
	}
}



int main(int argc, char **argv)
{
	if (argc != 3) {
		ROS_ERROR("Usage walls <candidateDist> <errThresh>");
		ROS_INFO("Setting default values...");
	} else {
		candidateDist = atoi(argv[1]);
		errThresh = atof(argv[2]);
	}
	ros::init(argc, argv, "walls");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, extendedSransac);
	ros::Publisher pub = n.advertise<red_fundamentals::Walls>("walls_topic", 1);

	while(ros::ok()) {
		ros::spinOnce();
		pub.publish(wall);
		ros::Duration(0.8).sleep();
	}
	return 0;
}
