#include <cstdlib>
#include "ros/ros.h"
#include "red_fundamentals/NewWalls.h"

using namespace std;

red_fundamentals::NewWalls walls;
int i = 0;

void set_walls(){
	// TODO Set mock wall output
	switch (i) {
		case 0:
			walls.L_A = {-0.4,0.4};
			walls.L_B = {0.4,0.4};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 1:
			walls.L_A = {-0.4,0.5};
			walls.L_B = {0.4,0.5};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;
		
		case 2:
			walls.L_A = {-0.5,0.4};
			walls.L_B = {0.3,0.4};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 3:
			walls.L_A = {-0.5,0.5};
			walls.L_B = {0.3,0.5};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 4:
			walls.L_A = {-0.2,0.4};
			walls.L_B = {0.2,0.4};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		default:
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

	}

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

	while (i < 10) {
		set_walls();
		pub.publish(walls);
		i++;
	}

	return 0;
}
