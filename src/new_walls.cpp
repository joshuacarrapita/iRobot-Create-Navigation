#include <cstdlib>
#include "ros/ros.h"
#include "red_fundamentals/NewWalls.h"

using namespace std;

red_fundamentals::NewWalls walls;
int i;

void set_walls(){

	// TODO Set mock wall output
	switch (i) {

		case 0:
			ROS_INFO("Case 0 reached with i = %d", i);
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {-0.4,0.4};
			walls.F_B = {0.4,0.4};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 1:
			ROS_INFO("Case 1 reached with i = %d", i);
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {-0.4,0.5};
			walls.F_B = {0.4,0.5};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;
		
		case 2:
			ROS_INFO("Case 2 reached with i = %d", i);
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {-0.5,0.4};
			walls.F_B = {0.3,0.4};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 3:
			ROS_INFO("Case 3 reached with i = %d", i);
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {-0.5,0.5};
			walls.F_B = {0.3,0.5};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 4:
			ROS_INFO("Case 4 reached with i = %d", i);
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {-0.2,0.4};
			walls.F_B = {0.2,0.4};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 5:
			ROS_INFO("Case 5 reached with i = %d", i);
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0.3,0.8};
			walls.R_B = {0.3,0.1};
			break;

		case 6:
			ROS_INFO("Case 6 reached with i = %d", i);
			walls.L_A = {-0.4,0.4};
			walls.L_B = {-0.4,1.2};
			walls.F_A = {0,0};
			walls.F_B = {0,0};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 7:
			ROS_INFO("Case 7 reached with i = %d", i);
			walls.L_A = {0,0};
			walls.L_B = {0,0};
			walls.F_A = {-0.3,1.8};
			walls.F_B = {0.5,1.8};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 8:
			ROS_INFO("Case 8 reached with i = %d", i);
			walls.L_A = {-0.4,0.4};
			walls.L_B = {-0.4,1.2};
			walls.F_A = {-0.3,1.8};
			walls.F_B = {0.5,1.8};
			walls.R_A = {0.3,0.8};
			walls.R_B = {0.3,0.1};
			break;

		case 9:
			ROS_INFO("Case 9 reached with i = %d", i);
			walls.L_A = {-0.4,0.4};
			walls.L_B = {-0.4,1.2};
			walls.F_A = {-0.3,1.8};
			walls.F_B = {0.5,1.8};
			walls.R_A = {0,0};
			walls.R_B = {0,0};
			break;

		case 10:
			ROS_INFO("Case 10 reached with i = %d", i);
			walls.L_A = {-0.3637,-0.21};
			walls.L_B = {-0.403,0.139};
			walls.F_A = {-0.4003,0.1457};
			walls.F_B = {0.3638,0.1934};
			walls.R_A = {0.3662,0.1866};
			walls.R_B = {0.3663,-0.2115};
			break;

		default:
			ROS_INFO("Case default reached with i = %d", i);
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
	
	i = 0;

	while (i < 12) {
		ros::Duration(1).sleep();
		set_walls();
		pub.publish(walls);
		ROS_INFO("Case %d published.", i);
		i++;
	}

	return 0;
}
