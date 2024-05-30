#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/Walls.h"
#include <cstdlib>

#define LARRSIZE 726
#define LARRRIGHTEND 272
#define LARRFRONTEND 498
#define LARROFFSET 44
#define LARRACTSIZE 682
#define ANGLESTEP 0.351906
#define LARRUSED 512
#define LARRUSEDOFFSET 85
#define ANGLESTEPUSED 0.351563
#define TOTALOFFSET 129

using namespace std;

double errThresh = 0.03;
int candidateDist = 5;
int winnerThresh = 50;

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
	int signer = 1;
	//if (pt_a.x < 0) { signer = -1; }
	double ydiff = pt_a.y-pt_b.y;
	return signer*((acos(ydiff/sqrt((pt_a.x-pt_b.x)*(pt_a.x-pt_b.x) + ydiff*ydiff))) * (180/(double) M_PI));
}

//calculates angle relative to y axis via slope of line
double calcAngleAlt(double slope)
{
	return (cos(slope)/sin(slope));
}

void resetMSG(red_fundamentals::Walls msg)
{
	//reset bools
	msg.walls[0] = false;
	msg.walls[1] = false;
	msg.walls[2] = false;
	//reset coords
	wall.r_wall_a[0] = 0.0;
	wall.r_wall_a[1] = 0.0;
	wall.r_wall_b[0] = 0.0;
	wall.r_wall_b[1] = 0.0;
	wall.f_wall_a[0] = 0.0;
	wall.f_wall_a[1] = 0.0;
	wall.f_wall_b[0] = 0.0;
	wall.f_wall_b[1] = 0.0;
	wall.l_wall_a[0] = 0.0;
	wall.l_wall_a[1] = 0.0;
	wall.l_wall_b[0] = 0.0;
	wall.l_wall_b[1] = 0.0;
	return;
}

//simplified RANSAC making use of environment constraints
void extendedSransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//array for holding valid coordinates
	coords coordinates[3][228] = {};
	coords pol;

	//0:right
	//1:front
	//2:left
	int lens[3] = {};

	//get all valid measurements and convert to cartesian
	for (int i = LARROFFSET; i < LARRRIGHTEND; i++) {
		if (isnan(msg->ranges[i]) && msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates[0][lens[0]] = polToCart(pol);
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", j, coordinates[j].x, coordinates[j].y);
		lens[0]++;
	}
	for (int i = LARRRIGHTEND; i < LARRFRONTEND; i++) {
		if (isnan(msg->ranges[i]) && msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates[1][frontlen] = polToCart(pol);
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", j, coordinates[j].x, coordinates[j].y);
		lens[1]++;
	}
	for (int i = LARRFRONTEND; i < LARRSIZE; i++) {
		if (isnan(msg->ranges[i]) && msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates[2][leftlen] = polToCart(pol);
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", j, coordinates[j].x, coordinates[j].y);
		lens[2]++;
	}
	//actual SRANSAC start
	coords winner_a = robotpos;
	coords winner_b = robotpos;
	int curInliers;
	int maxInliers;
	//look for walls in each respective section of laser values
	for (int k = 0; k < 3; k++) {
		maxInliers = 0;
		//ROS_INFO("looking for wall %i", k);
		//take candidates with fixed interval instead of random
		for (int i = 0; i < lens[k]-(candidateDist+1); i+=candidateDist) {
			//ROS_INFO("checking candidates %i", i);
			curInliers = 0;
			coords candidate_a = coordinates[k][i];
			coords candidate_b = coordinates[k][i+candidateDist];
			//count number of inliers
			for (int j = 0; j < lens[k]; j++) {
				if (calcDist(candidate_a, candidate_b, coordinates[j]) < errThresh) {
					curInliers++;
				}
			}
			//update winner if current candidates have more inliers than previous best and threshold
			if (curInliers > maxInliers && curInliers > winnerThresh) {
				maxInliers = curInliers;
				winner_a = candidate_a;
				winner_b = candidate_b;
			}
		}

		if (winner_a != robotpos) {
			ROS_INFO("Wall %d found\n",k);
			wall.walls[k] = true;
			switch(k)
			{
				case 0:
					ROS_INFO("Right wall detected\n");
					wall.r_wall_a[0] = winner_b.x;
					wall.r_wall_a[1] = winner_b.y;
					wall.r_wall_b[0] = winner_a.x;
					wall.r_wall_b[1] = winner_a.y;
				case 1:
					ROS_INFO("Front wall detected\n");
					wall.f_wall_a[0] = winner_b.x;
					wall.f_wall_a[1] = winner_b.y;
					wall.f_wall_b[0] = winner_a.x;
					wall.f_wall_b[1] = winner_a.y;
				case 2:
					ROS_INFO("Left wall detected\n");
					wall.l_wall_a[0] = winner_b.x;
					wall.l_wall_a[1] = winner_b.y;
					wall.l_wall_b[0] = winner_a.x;
					wall.l_wall_b[1] = winner_a.y;

		}
		//ROS_INFO("Best wall %i with %i inliers has dist: %.7lf angle: %.7lf", k, maxInliers, wall.dist[k], wall.angle[k]);
		maxInliers = 0;
	}
}



int main(int argc, char **argv)
{
	if (argc != 4) {
		ROS_ERROR("Usage: walls <candidateDist> <errThresh> <winnerThresh>");
		ROS_INFO("Setting default values...");
	} else {
		candidateDist = atoi(argv[1]);
		errThresh = atof(argv[2]);
		winnerThresh = atoi(argv[3]);
	}
	ros::init(argc, argv, "walls");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, extendedSransac);
	ros::Publisher pub = n.advertise<red_fundamentals::Walls>("walls_topic", 1);
	ROS_INFO("start detecting walls");
	while(ros::ok()) {
		ros::spinOnce();
		pub.publish(wall);
		ros::Duration(0.2).sleep();
	}
	return 0;
}
