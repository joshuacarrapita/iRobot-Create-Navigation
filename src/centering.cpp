#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "sensor_msgs/LaserScan.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;
int wall;
int isPerpendicular;
int distCorrect;
int turnCtr;
int centeringPhase;		//0 if still orienting to first wall; 1 if orienting to second wall
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
}

//placeholder function to turn left/right by 90°
//TODO: replace with exact turning function from square task
void turnRightAngle(int direction)
{
	stop();
	if (direction) {
		srv.request.left = 6;
		srv.request.right = -6;
		diffDrive.call(srv);
		ros::Duration(0.9).sleep();
		stop();
	} else {
		srv.request.left = -6;
		srv.request.right = 6;
		ros::Duration(0.9).sleep();
		stop();
	}
}

/*returns integer depending on which wall is detected first
 * 0 = no wall
 * 1 = front
 * 2 = right
 * 3 = back
 * 4 = left
 */
int findWall(red_fundamentals::WallInfo::ConstPtr& msg)
{
	double walls[4] = msg; //need to add sizeof(header) to addr

	for (int i = ; i < 4; i++) {
		if (walls[i] > 0) {
			wall = i + 1;
		} 
	}
	return wall;
}

//turns robot until it is roughly perpendicular to found wall
void perpendicularise(red_fundamentals::WallInfo::ConstPtr& msg)
{
	double angles[4] = (msg + sizeof(double) * 4); //need to add sizeof(header) to addr
	if (angles[wall] > 0 + tolerance) {
		srv.request.left = speed;
		srv.request.right = -speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	} else if (angles[wall] < 0 - tolerance) {
		srv.request.left = -speed;
		srv.request.right = speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	} else {
		isPerpendicular = 1;
		heading = 0;
	}
	return;
}

//moves robot until it is
void correctDist(red_fundamentals::WallInfo::ConstPtr& msg)
{
	double walls[4] = msg; //need to add sizeof(header) to addr
	if (walls[wall] > 25 + tolerance) {
		srv.request.left = speed;
		srv.request.right = speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	} else if (walls[wall] < 25 - tolerance) {
		srv.request.left = -speed;
		srv.request.right = -speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	} else {
		distCorrect = 1;
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
	if (!wall) {
		if (!findWall(msg) && turnCtr < 4) {
			turnRightAngle(1);
			turnCtr++;
		}
		if (!wall && turnCtr > 3) {
			srv.request.left = 5;
			srv.request.right = 5;
			diffDrive.call(srv);
			ros::Duration(moveIncrement).sleep();
			stop();
		}
	}
	//turn robot to be perpendicular to detected wall
	if (wall && !isPerpendicular) {
		perpendicularise(msg);
	}
	//correct distance
	if (isPerpendicular && !distCorrect) {
		correctDist(msg);
	}
	//find second wall and move to phase 2
	if (distCorrect) {
		if (centeringPhase == 0 && heading == 0) {
			turnRightAngle(1)
			heading = 1;
			if (findWall(msg) == 1) {
				isPerpendicular == 0;
				distCorrect == 0;
				centeringPhase == 1;
			}
			return;
		} else if (heading == 1) {
			turnRightAngle(1);
			turnRightAngle(1);
			heading = 3;
			if (findWall(msg) == 1) {
				isPerpendicular == 0;
				distCorrect == 0;
				centeringPhase == 1;
			}
			return;
		}
		srv.request.left = 5;
		srv.request.right = 5;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		if (findWall(msg) == 1)
			isPerpendicular == 0;
			distCorrect == 0;
			centeringPhase == 1;
	}
	//ROS_INFO("%f", msg->);
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
	//ros::spin();
	return 0;

}
