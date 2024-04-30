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
int centeringPhase;		//0 if still orienting to first wall; 1 if orienting to second wall
double tolerance = 3;		//tolerance value for angle to wall
double speed = 5;		//motor speed
double moveIncrement = 0.3;	//time to move each increment


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
		if (walls[4] > 0) {
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
		srv.request.left = 0;
		srv.request.right = 0;
		diffDrive.call(srv);
		return;
	} else if (angles[wall] < 0 - tolerance) {
		srv.request.left = -speed;
		srv.request.right = speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		srv.request.left = 0;
		srv.request.right = 0;
		diffDrive.call(srv);
		return;
	} else {
		isPerpendicular = 1;
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
		srv.request.left = 0;
		srv.request.right = 0;
		diffDrive.call(srv);
		return;
	} else if (walls[wall] < 25 - tolerance) {
		srv.request.left = -speed;
		srv.request.right = -speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		srv.request.left = 0;
		srv.request.right = 0;
		diffDrive.call(srv);
		return;
	} else {
		distCorrect = 1;
		return;
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
	//check if wall in sight
	if (!wall) {
		findWall(msg);
	}
	//turn robot to be perpendicular to detected wall
	if (wall && !isPerpendicular) {
		perpendicularise(msg);
	}
	//correct distance
	if (isPerpendicular && !distCorrect) {
		correctDist(msg);
	}
	ROS_INFO("%f", msg->);
}

int main(int argc, char **argv)
{
	if (argc != 4) {
		ROS_ERROR("Usage: ./centering tolerance speed moveIncrement");
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
