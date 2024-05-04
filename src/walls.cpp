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

struct coords
{
	double x;	//for polar = r
	double y;	//for polar = angle
} myCoords;


//converts polar to cart coordinates
struct coords polToCart(struct coords polar)
{
	//ROS_INFO("Converting polar coordinate with r = %f and angle = %f", polar.x, polar.y);
	struct coords cart;
	cart.x = polar.x * cos(polar.y);
	cart.y = polar.x * sin(polar.y);
	//ROS_INFO("To cartesian coordinate with (%f , %f)", cart.x, cart.y);
	return cart;
}

float calcAngle(int index)
{
	return((index * ANGLESTEPUSED/180)* ((double) M_PI));
}

void sransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	vector<coords> coordinates;
	struct coords pol;
	//int j = 0;
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
	for (int i = 0; i < coordinates.size(); i++) {
	
	}
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
