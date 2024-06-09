#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/WallsAlt.h"
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
int winnerThresh = 70;

ros::Publisher pub;
red_fundamentals::WallsAlt wall;

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
	vector<coords> coordinates;
	coordinates.reserve(683);
	coords pol;

	//get all valid measurements and convert to cartesian
	for (int i = LARROFFSET; i < LARRSIZE; i++) {
		if (isnan(msg->ranges[i]) | msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates.push_back(polToCart(pol));
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", lens[0], coordinates[0][(lens[0])].x, coordinates[0][(lens[0])].y);
	}

	//actual SRANSAC start
	int curInliers;
	int maxInliers;

	vector<coords> outliers;
	outliers.reserve(683);
	vector<coords> winneroutliers;
	winneroutliers.reserve(683);
	vector<coords> inliers;
	inliers.reserve(683);

	coords winners[3][2];
	int winnerInliers[3] = {};

	//look for 3 best walls
	for (int k = 0; k < 3; k++) {
		//if not enough candidates stop looking for walls
		if (coordinates.size() < winnerThresh) {
			//ROS_INFO("not enough candidates");
			break;
		}
		coords winner_a = robotpos;
		coords winner_b = robotpos;
		maxInliers = 0;
		//ROS_INFO("looking for wall %i with %d candidates", k,coordinates.size());
		//take candidates with fixed interval instead of random
		for (int i = 0; i < coordinates.size()-(candidateDist+1); i+=candidateDist) {
			//ROS_INFO("checking candidates %i", i);
			curInliers = 0;
			coords candidate_b = coordinates[i];
			coords candidate_a = coordinates[i+candidateDist];
			//count number of inliers
			for (int j = 0; j < coordinates.size(); j++) {
				if (calcDist(candidate_a, candidate_b, coordinates[j]) < errThresh) {
					curInliers++;
					inliers.push_back(coordinates[j]);
				} else {
					outliers.push_back(coordinates[j]);
				}
			}
			//update winner if current candidates have more inliers than previous best and threshold and remove inliers
			if (curInliers > maxInliers && curInliers > winnerThresh) {
				maxInliers = curInliers;
				winneroutliers = outliers;
				winner_a = inliers.back();
				winner_b = inliers.front();
			}
			outliers.clear();
			inliers.clear();
		}
		if (maxInliers > winnerThresh) {
			//ROS_INFO("Wall %d found",k);
			winners[k][0] = winner_a;
			winners[k][1] = winner_b;
			winnerInliers[k] = maxInliers;
		}
		coordinates = winneroutliers;
	}
	//classify walls
	for (int i = 0; i < 3; i++) {
		if (winnerInliers[i]) {
			double ang = calcAngle(winners[i][0],winners[i][1]);
			//ROS_INFO("Wall %d has angle: %lf",i,ang);
			if (ang > 120) {
				//ROS_INFO("Left wall found");
				if (winners[i][0].y < 0.05) {
					wall.walls[2] = true;
				}
				wall.L_A[0] = winners[i][0].x;
				wall.L_A[1] = winners[i][0].y;
				wall.L_B[0] = winners[i][1].x;
				wall.L_B[1] = winners[i][1].y;
			} else if (ang < 60) {
				//ROS_INFO("Right wall found");
				if (winners[i][1].y < 0.05) {
					wall.walls[0] = true;
				}
				wall.R_A[0] = winners[i][0].x;
				wall.R_A[1] = winners[i][0].y;
				wall.R_B[0] = winners[i][1].x;
				wall.R_B[1] = winners[i][1].y;
			} else {
				//ROS_INFO("Front wall found");
				if (winners[i][0].y < 0.4) {
					wall.walls[1] = true;
				}
				wall.F_A[0] = winners[i][0].x;
				wall.F_A[1] = winners[i][0].y;
				wall.F_B[0] = winners[i][1].x;
				wall.F_B[1] = winners[i][1].y;
			}

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
	pub = n.advertise<red_fundamentals::WallsAlt>("walls_alt_topic", 1);
	ROS_INFO("start detecting walls");
	while(ros::ok()) {
		ros::spinOnce();
		//ros::Duration(0.5).sleep();
	}
	return 0;
}
