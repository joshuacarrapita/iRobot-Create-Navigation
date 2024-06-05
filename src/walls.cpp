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

double errThresh = 0.02;
int candidateDist = 5;
int winnerThresh = 160;

ros::Publisher pub;
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

void resetMSG()
{
	//ROS_INFO("Resetting msg vals");
	//reset bools
	wall.walls[0] = false;
	wall.walls[1] = false;
	wall.walls[2] = false;
	//reset coords
	wall.R_A[0] = 0.0;
	wall.R_A[1] = 0.0;
	wall.R_B[0] = 0.0;
	wall.R_B[1] = 0.0;
	wall.F_A[0] = 0.0;
	wall.F_A[1] = 0.0;
	wall.F_B[0] = 0.0;
	wall.F_B[1] = 0.0;
	wall.L_A[0] = 0.0;
	wall.L_A[1] = 0.0;
	wall.L_B[0] = 0.0;
	wall.L_B[1] = 0.0;
	return;
}

//simplified RANSAC making use of environment constraints
void extendedSransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//reset walls msg contents
	resetMSG();

	//array for holding valid coordinates
	coords coordinates[3][228] = {};
	coords pol;

	//0:right
	//1:front
	//2:left
	int lens[3] = {};

	//get all valid measurements and convert to cartesian
	for (int i = LARROFFSET; i < LARRRIGHTEND; i++) {
		if (isnan(msg->ranges[i]) | msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates[0][(lens[0])] = polToCart(pol);
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", lens[0], coordinates[0][(lens[0])].x, coordinates[0][(lens[0])].y);
		lens[0]++;
	}
	for (int i = LARRRIGHTEND; i < LARRFRONTEND; i++) {
		if (isnan(msg->ranges[i]) | msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates[1][(lens[1])] = polToCart(pol);
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", lens[1], coordinates[1][(lens[1])].x, coordinates[1][(lens[1])].y);
		lens[1]++;
	}
	for (int i = LARRFRONTEND; i < LARRSIZE; i++) {
		if (isnan(msg->ranges[i]) | msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates[2][(lens[2])] = polToCart(pol);
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", lens[2], coordinates[2][(lens[2])].x, coordinates[2][(lens[2])].y);
		lens[2]++;
	}

	//actual SRANSAC start
	int curInliers;
	int maxInliers;

	coords winners[3][2];
	int winnerInliers[3] = {};

	//look for walls in each respective section of laser values
	for (int k = 0; k < 3; k++) {
		coords winner_a = robotpos;
		coords winner_b = robotpos;
		maxInliers = 0;
		//ROS_INFO("looking for wall %i", k);
		//take candidates with fixed interval instead of random
		for (int i = 0; i < lens[k]-(candidateDist+1); i+=candidateDist) {
			//ROS_INFO("checking candidates %i", i);
			curInliers = 0;
			coords candidate_b = coordinates[k][i];
			coords candidate_a = coordinates[k][i+candidateDist];
			//count number of inliers
			for (int j = 0; j < lens[k]; j++) {
				if (calcDist(candidate_a, candidate_b, coordinates[k][j]) < errThresh) {
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

		if (maxInliers > winnerThresh) {
			//ROS_INFO("Wall %d found",k);
			switch(k)
			{
				case 0:
					//ROS_INFO("Right wall detected at A: (%lf,%lf) B: (%lf,%lf) with %d inliers", winner_a.x,winner_a.y,winner_b.x,winner_b.y,maxInliers);
					wall.walls[0] = true;
					winners[0][0] = winner_a;
					winners[0][1] = winner_b;
					wall.R_A[0] = winner_a.x;
					wall.R_A[1] = winner_a.y;
					wall.R_B[0] = winner_b.x;
					wall.R_B[1] = winner_b.y;
					winnerInliers[0] = maxInliers;
					continue;
				case 1:
					//ROS_INFO("Front wall detected at A: (%lf,%lf) B: (%lf,%lf) with %d inliers", winner_a.x,winner_a.y,winner_b.x,winner_b.y,maxInliers);
					wall.walls[1] = true;
					winners[1][0] = winner_a;
					winners[1][1] = winner_b;
					wall.F_A[0] = winner_a.x;
					wall.F_A[1] = winner_a.y;
					wall.F_B[0] = winner_b.x;
					wall.F_B[1] = winner_b.y;
					winnerInliers[1] = maxInliers;
					continue;
				case 2:
					//ROS_INFO("Left wall detected at A: (%lf,%lf) B: (%lf,%lf) with %d inliers", winner_a.x,winner_a.y,winner_b.x,winner_b.y,maxInliers);
					wall.walls[2] = true;
					winners[2][0] = winner_a;
					winners[2][1] = winner_b;
					wall.L_A[0] = winner_a.x;
					wall.L_A[1] = winner_a.y;
					wall.L_B[0] = winner_b.x;
					wall.L_B[1] = winner_b.y;
					winnerInliers[2] = maxInliers;
					continue;
			}
		}

	}
	//check if duplicate wall detected then choose wall with more inliers
	if (calcDist(winners[1][0], winners[1][1], winners[0][0]) < 0.05 && calcDist(winners[1][0], winners[1][1], winners[0][1]) < 0.05) {
		if (winnerInliers[0] <= winnerInliers[1]) {
			//ROS_INFO("Right wall same as Front. Deleting...");
			wall.R_A[0] = 0.0;
			wall.R_A[1] = 0.0;
			wall.R_B[0] = 0.0;
			wall.R_B[1] = 0.0;
			wall.walls[0] = false;
		} else {
			//ROS_INFO("Front wall same as Right. Deleting...");
			wall.F_A[0] = 0.0;
			wall.F_A[1] = 0.0;
			wall.F_B[0] = 0.0;
			wall.F_B[1] = 0.0;
			wall.walls[1] = false;
		}
	} else if (calcDist(winners[1][0], winners[1][1], winners[2][0]) < 0.05 && calcDist(winners[1][0], winners[1][1], winners[2][1]) < 0.05) {
		if (winnerInliers[2] <= winnerInliers[0]) {
			//ROS_INFO("Left wall same as Front. Deleting...");
			wall.L_A[0] = 0.0;
			wall.L_A[1] = 0.0;
			wall.L_B[0] = 0.0;
			wall.L_B[1] = 0.0;
			wall.walls[2] = false;
		} else {
			//ROS_INFO("Front wall same as Left. Deleting...");
			wall.F_A[0] = 0.0;
			wall.F_A[1] = 0.0;
			wall.F_B[0] = 0.0;
			wall.F_B[1] = 0.0;
			wall.walls[1] = false;
		}
	}
	pub.publish(wall);
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
	pub = n.advertise<red_fundamentals::Walls>("walls_topic", 1);
	ROS_INFO("start detecting walls");
	while(ros::ok()) {
		ros::spinOnce();
		//ros::Duration(0.5).sleep();
	}
	return 0;
}


