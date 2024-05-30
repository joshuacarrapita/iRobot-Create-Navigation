#include "ros/ros.h"
#include "red_fundamentals/NewWalls.h"
#include <cstdlib>

using namespace std;

red_fundamentals::NewWalls walls;

void set_walls(){
	// TODO Set mock wall output
	walls.L_A = 0;
	walls.L_B = 0;
	walls.F_A = 0;
	walls.F_B = 0;
	walls.R_A = 0;
	walls.R_B = 0;
}


int main(int argc, char **argv)
{
	if (argc != 1) {
		ROS_ERROR("Usage: new_walls");
		ROS_INFO("Setting default values...");
	}

	ros::init(argc, argv, "new_walls");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<red_fundamentals::NewWalls>("new_walls_topic", 1);

	while(ros::ok()) {
		set_walls();
		pub.publish(walls);
	}

	return 0;
}
