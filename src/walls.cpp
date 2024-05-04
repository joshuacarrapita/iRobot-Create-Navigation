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

red_fundamentals::Walls wall;

struct coords
{
	float x;	//for polar = r
	float y;	//for polar = angle
} myCoords;


//converts polar to cart coordinates
struct coords polToCart(coords polar)
{
	struct coords cart;
	cart.x = polar.x * cos(polar.y);
	cart.y = polar.x * sin(polar.y);
	return cart;
}

float calcAngle(int index)
{
	return index * ANGLESTEPUSED;
}

void sransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	struct coords coordinates[LARRUSED];
	struct coords pol;
	for (int i = (TOTALOFFSET-1); i < LARRSIZE - LARRUSEDOFFSET; i++) {
		pol.x = msg->ranges[i];
		pol.y = calcAngle(i - (TOTALOFFSET-1));
		coordinates[i] = polToCart(pol);
		ROS_INFO("Generated cartesian coordinate with (%f , %f)", coordinates[i].x, coordinates[i].y);
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
		pub.publish(wall);
		ros::Duration(3).sleep();
	}
	return 0;
}
