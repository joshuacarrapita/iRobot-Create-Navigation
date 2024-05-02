#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "sensor_msgs/LaserScan.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

//int wall; //probably deprecated since msg redesign
int isPerpendicular;
int distCorrect;
int turnCtr;
int centeringPhase;		//0 if still orienting to first wall; 1 if orienting to second wall; 2 if centered
int heading;			//current heading of robot; 0 = north, 1 = east, 2 = south, 3 = west

double tolerance = 3;		//tolerance value for angle to wall
double speed = 5;		//motor speed
double moveIncrement = 0.3;	//time to move each increment

//stop robot
void stop()
{
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);
	return;
}

//placeholder function to turn left/right by 90°
//set direction to 1 if right, 0 if left
//TODO: replace with exact turning function from square task
void turnRightAngle(int direction)
{
	stop();
	//turn right
	if (direction) {
		srv.request.left = 6;
		srv.request.right = -6;
		diffDrive.call(srv);
		ros::Duration(0.9).sleep();
		stop();
		return;
	//turn left
	} else {
		srv.request.left = -6;
		srv.request.right = 6;
		ros::Duration(0.9).sleep();
		stop();
	}
	return;
}


/*returns integer depending on which wall is detected first
 * 0 = no wall
 * 1 = front
 * 2 = right
 * 3 = back
 * 4 = left
 */
/* probably deprecated since msg redesign
int findWall(red_fundamentals::WallInfo::ConstPtr& msg)
{
	double walls[4] = msg; //need to add sizeof(header) to addr

	for (int i = ; i < 4; i++) {
		if (walls[i] > 0) {
			wall = i + 1;
		} 
	}
	return wall;
}*/

//turns robot until it is roughly perpendicular to found wall
void perpendicularise(red_fundamentals::WallInfo::ConstPtr& msg)
{
	//double angles[4] = (msg + sizeof(double) * 4); //need to add sizeof(header) to addr
	//turn right if wall right of robot
	if (msg->angle > 0 + tolerance) {
		srv.request.left = speed;
		srv.request.right = -speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	//turn left if wall left of robot
	} else if (msg->angle < 0 - tolerance) {
		srv.request.left = -speed;
		srv.request.right = speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	//set variables if perpendicular
	} else {
		isPerpendicular = 1;
		heading = 0;
	}
	return;
}

//moves robot until distance to wall is half cell width/length
void correctDist(red_fundamentals::WallInfo::ConstPtr& msg)
{
	//double walls[4] = msg; //need to add sizeof(header) to addr
	//if too far away move forward
	if (msg->wall > 25 + tolerance) {
		srv.request.left = speed;
		srv.request.right = speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	//if too close move back
	} else if (msg->wall < 25 - tolerance) {
		srv.request.left = -speed;
		srv.request.right = -speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	//if correct distance, set variables
	} else {
		distCorrect = 1;
		if (centeringPhase == 1) {
			centeringPhase = 2;	//finished centering
		}
	}
	return;
}

void center(const red_fundamentals::WallInfo::ConstPtr& msg)
{
	//TODO: implement function that uses data from the wall detection service to center the robot within a cell
	/*idea: find wall -> adjust robot to be perpendicular to wall
         *-> adjust distance so robot is 25cm away from wall (laser sensor is 15cm from center of robot)
	 *->turn right/left by 90° -> drive forward until you find other wall ahead
	 *->adjust so robot is 25cm away from new wall
	 *(optionally: track how many cells you moved from initial point and retrace)
	 *->robot should be centered now
	 */

	//check if wall in sight else move until it is
	if (!msg->wall) {
		//if no wall in sight turn right until full 360° rotation
		if (!msg->wall && turnCtr < 4) {
			turnRightAngle(1);
			turnCtr++;
			return;
		//if after turning still no wall move forwards until wall found
		} else {
			srv.request.left = 5;
			srv.request.right = 5;
			diffDrive.call(srv);
			ros::Duration(moveIncrement).sleep();
			stop();
			return;
		}
	}
	//turn robot to be perpendicular to detected wall
	if (wall && !isPerpendicular) {
		perpendicularise(msg);
		return;
	}
	//correct distance
	if (isPerpendicular && !distCorrect) {
		correctDist(msg);
		return;
	}
	//find second wall and move to phase 2
	if (distCorrect) {
		//after finished adjusting robot to firs wall
		if (centeringPhase == 0 && heading == 0) {
			//turn right by 90° and look for wall in front
			turnRightAngle(1)
			heading = 1;
			//if found move on to phase 2
			if (findWall(msg) == 1) {
				isPerpendicular == 0;
				distCorrect == 0;
				centeringPhase == 1;
			}
			return;
		//if right of first wall no wall turn right by 180° and look for wall in front
		} else if (heading == 1) {
			turnRightAngle(1);
			turnRightAngle(1);
			heading = 3;
			//if found move on to phase 2
			if (findWall(msg) == 1) {
				isPerpendicular == 0;
				distCorrect == 0;
				centeringPhase == 1;
			}
			return;
		}
		//if no suitable walls found directly connected to first wall, move forward with first wall left of robot and look for wall in front
		srv.request.left = 5;
		srv.request.right = 5;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		//if found move on to phase 2
		if (findWall(msg) == 1)
			isPerpendicular == 0;
			distCorrect == 0;
			centeringPhase == 1;
	}
	return;
}

int main(int argc, char **argv)
{
	if (argc != 4) {
		ROS_ERROR("Usage: centering <tolerance> <speed> <moveIncrement>");
		return 0;
	}
	tolerance = argv[0];
	speed = argv[1];
	moveIncrement = argv[2];
	ros::init(argc, argv, "centering");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("wall_info", 1, center);
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	//spin until centered
	while (centeringPhase != 2) {
		ros::spinOnce();
	}
	return 0;

}
