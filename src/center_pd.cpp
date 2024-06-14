#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "red_fundamentals/WallsCombined.h"
#include "paradriver.hpp"
#include "center.hpp"


Center::Center() : centeringPhase(LOOKINGFORFIRSTWALL), heading(-1), turnCtr(0), tolerance(0.03)
{
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    sub = n.subscribe("walls_topic", 1, &Center::center, this);
}

//turns robot until it is roughly perpendicular to found wall
void Center::perpendicularise(const red_fundamentals::WallsCombined::ConstPtr& msg)
{   

	//set variables if perpendicular
	if (msg->angle < 90 + tolerance*20 && msg->angle > 90 - tolerance*20) {
		ROS_INFO("already perpendicular...");
		return;
	//turn right if wall right of robot
	} else if (msg->angle < 90) {
		ROS_INFO("turning right by %f degrees", (90-msg->angle));
		driver.turn(90-msg->angle);
		return;
	//turn left if wall left of robot
	} else if (msg->angle > 90) {
		ROS_INFO("turning left by %f degrees", (msg->angle-90));
		driver.turn(-(msg->angle-90));
		return;
	}
	return;
}

//moves robot until distance to wall is half cell width/length
void Center::correctDist(const red_fundamentals::WallsCombined::ConstPtr& msg)
{
	//if too far away move forward
	if (msg->dist > 0.25) {
		ROS_INFO("moving forward to correct distance");
		driver.moveDistEnc(msg->dist - 0.25);
		return;
	//if too close move back
	} else if (msg->dist < 0.25) {
		ROS_INFO("moving backwards to correct distance");
		moveDist(-(0.25 - msg->dist));
		return;
	}
	return;
}

//centers the robot in first available cell with 2 perpendicular walls
void Center::center(const red_fundamentals::WallsCombined::ConstPtr& msg)
{
	//check if wall in sight else move until it is
	if (centeringPhase == LOOKINGFORFIRSTWALL && !msg->dist) {
		//if no wall in sight turn right until full 360° rotation
		if (turnCtr < 4) {
			ROS_INFO("turning to find first wall");
			driver.turn(90);
			turnCtr++;
			return;
		//if after turning still no wall move forwards until wall found
		} else {
			ROS_INFO("moving to find first wall");
			srv.request.left = 5;
			srv.request.right = 5;
			diffDrive.call(srv);
			return;
		}
	}
	//turn robot to be perpendicular to detected first wall
	if (centeringPhase == LOOKINGFORFIRSTWALL && msg->dist) {
		driver.stop();
		ROS_INFO("perpendicularising");
		perpendicularise(msg);
		centeringPhase = PERPENDICULARTOFIRSTWALL;
		return;
	}
	//correct distance
	if (centeringPhase == PERPENDICULARTOFIRSTWALL && msg->dist) {
		correctDist(msg);
		centeringPhase = DISTANCEDTOFIRSTWALL;
		heading = 0;
		return;
	}
	//find second wall and move to phase 2
	if (centeringPhase == DISTANCEDTOFIRSTWALL) {
		//after finished adjusting robot to first wall turn right by 90° and look for wall in front
		if (heading == 0) {
			ROS_INFO("turning right to find second wall");
			turn(90);
			centeringPhase = LOOKINGFORSECONDWALL;
			heading = 1;
			ros::Duration(0.3).sleep();
			return;
		}
		//if found move on to phase 2
		if (heading == 1 && msg->dist < 0.8 && msg->angle > 75 && msg->angle < 105) {
			ROS_INFO("second wall found at heading 1");
			if (centeringPhase == LOOKINGFORSECONDWALL) {
				perpendicularise(msg);
				centeringPhase = PERPENDICULARTOSECONDWALL;
			} else if (centeringPhase == PERPENDICULARTOSECONDWALL) {
				correctDist(msg);
				centeringPhase = CENTERED;
			}
			return;
		//if right of first wall no wall turn left by 180° and look for wall in front
		} else if (heading != 3) {
			ROS_INFO("turning 180 deg more to find second wall left of origin");
			driver.turn(-180);
			heading = 3;
			ros::Duration(0.3).sleep();
			return;
		}
		//if found move on to phase 2
		if (heading == 3 && msg->dist < 0.8 && msg->angle > 75 && msg->angle < 105) {
			ROS_INFO("second wall found at heading 3 with dist: %f and angle %f", msg->dist, msg->angle);
			driver.stop();
			if (centeringPhase == LOOKINGFORSECONDWALL) {
				perpendicularise(msg);
				centeringPhase = PERPENDICULARTOSECONDWALL;
			} else if (centeringPhase == PERPENDICULARTOSECONDWALL) {
				correctDist(msg);
				centeringPhase = CENTERED;
			}
			return;
		} else {
			//if no suitable walls found directly connected to first wall, move forward with first wall left of robot and look for wall in front
			ROS_INFO("no second wall found looking for new first wall in adjacent cell");
			srv.request.left = 5;
			srv.request.right = 5;
			diffDrive.call(srv);
			centeringPhase = LOOKINGFORFIRSTWALL;
			return;
		}
		
	}
	return;
}

void Center::centering(){
	while (ros::ok() && centeringPhase != CENTERED) {
		ros::spinOnce();
	}
	ROS_INFO("centering finished");
	return;
}

/* int main(int argc, char **argv)
{
    ros::init(argc, argv, "center");
	ros::NodeHandle n;
	Center c;

    c.centering();
}
 */
