#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "red_fundamentals/Walls.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

//int wall; //probably deprecated since msg redesign
int isPerpendicular;
int distCorrect;
int turnCtr;
int noPerp;
int centeringPhase;		//0 if still orienting to first wall; 1 if orienting to second wall; 2 if centered
int heading;			//current heading of robot; 0 = north, 1 = east, 2 = south, 3 = west

double tolerance = 0.03;	//tolerance value for angle to wall
double speed = 4;		//motor speed
double moveIncrement = 0.1;	//time to move each increment

//stop robot
void stop()
{
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);
	return;
}

void moveDist(int direction, float distance)
{
	double md_speed = 6.4;
	if (direction < 0) {
		md_speed = -md_speed;
	}
	srv.request.left = md_speed;
	srv.request.right = md_speed;
	diffDrive.call(srv);
	ros::Duration(distance*5).sleep();
	stop();
	return;
}

//placeholder function to turn left/right by 90°
//set direction to 1 if right, 0 if left
//TODO: replace with exact turning function from square task
void turnRightAngle(int direction)
{
	stop();
	//turn right
	if (direction > 0) {
		srv.request.left = 6.3;
		srv.request.right = -6.3;
		diffDrive.call(srv);
		ros::Duration(1.0).sleep();
		stop();
		return;
	//turn left
	} else if (direction < 0) {
		srv.request.left = -6.3;
		srv.request.right = 6.3;
		ros::Duration(1.0).sleep();
		stop();
		return;
	}
	return;
}

void turnAngle(int direction, double angle)
{
	stop();
	float a_speed = 3.2;
	double anglemapped = (angle*2.0)/90;
	if (!direction) { a_speed = -a_speed; }
	srv.request.left =  a_speed;
	srv.request.right = -a_speed;
	diffDrive.call(srv);
	ros::Duration(anglemapped).sleep();
	stop();
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
void perpendicularise(const red_fundamentals::Walls::ConstPtr& msg)
{
	//set variables if perpendicular
	if (msg->angle < 85 + tolerance*20 && msg->angle > 85 - tolerance*20) {
		isPerpendicular = 1;
		heading = 0;
		return;
	//turn right if wall right of robot
	} else if (msg->angle > 85 + tolerance*15) {
		srv.request.left = speed;
		srv.request.right = -speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	//turn left if wall left of robot
	} else if (msg->angle < 88 - tolerance*15) {
		srv.request.left = -speed;
		srv.request.right = speed;
		diffDrive.call(srv);
		ros::Duration(moveIncrement).sleep();
		stop();
		return;
	}
	return;
}

//moves robot until distance to wall is half cell width/length
void correctDist(const red_fundamentals::Walls::ConstPtr& msg)
{
	//if too far away move forward
	if (msg->dist > 0.25 + tolerance) {
		ROS_INFO("moving forward to correct distance");
		moveDist(1, msg->dist - 0.25);
		distCorrect = 1;
		if (centeringPhase == 1) {
			centeringPhase = 2;
		}
		return;
	//if too close move back
	} else if (msg->dist < 0.25 - tolerance) {
		ROS_INFO("moving backwards to correct distance");
		moveDist(-1, 0.25 - msg->dist);
		distCorrect = 1;
		if (centeringPhase == 1) {
			centeringPhase = 2;
		}
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

void center(const red_fundamentals::Walls::ConstPtr& msg)
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
	if (!msg->dist) {
		//if no wall in sight turn right until full 360° rotation
		if (turnCtr < 4) {
			ROS_INFO("turning to find first wall");
			turnRightAngle(1);
			turnCtr++;
			return;
		//if after turning still no wall move forwards until wall found
		} else {
			ROS_INFO("moving to find first wall");
			srv.request.left = 5;
			srv.request.right = 5;
			diffDrive.call(srv);
			ros::Duration(moveIncrement).sleep();
			stop();
			return;
		}
	}
	//turn robot to be perpendicular to detected wall
	if (msg->dist && !isPerpendicular) {
		ROS_INFO("perpendicularising");
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
		if (heading == 0) {
			ROS_INFO("turning right to find second wall");
			//turn right by 90° and look for wall in front
			turnRightAngle(1);
			heading = 1;
			return;
		}
		//if found move on to phase 2
		if (heading == 1 && msg->dist && msg->angle > 75 && msg->angle < 105) {
			ROS_INFO("second wall found at heading 1");
			isPerpendicular = 0;
			distCorrect = 0;
			centeringPhase = 1;
			return;
		//if right of first wall no wall turn right by 180° and look for wall in front
		} else if (heading != 3) {
			ROS_INFO("turning 180 deg more to find second wall left of origin");
			turnRightAngle(1);
			turnRightAngle(1);
			heading = 3;
			return;
		}
		//if found move on to phase 2
		if (msg->dist < 0.8 && heading == 3 && msg->angle < 105 && msg->angle > 75) {
			ROS_INFO("second wall found at heading 3 with dist: %f and angle %f", msg->dist, msg->angle);
			isPerpendicular = 0;
			distCorrect = 0;
			centeringPhase = 1;
			return;
		} else {
			ROS_INFO("moving to find second wall");
			//if no suitable walls found directly connected to first wall, move forward with first wall left of robot and look for wall in front
			srv.request.left = 5;
			srv.request.right = 5;
			diffDrive.call(srv);
			ros::Duration(moveIncrement).sleep();
			stop();
			return;
		}
		//if found move on to phase 2
		if (msg->dist && msg->angle <135) {
			ROS_INFO("moving on to phase 2");
			isPerpendicular = 0;
			distCorrect = 0;
			centeringPhase = 1;
		}
	}
	return;
}

int main(int argc, char **argv)
{
	if (argc != 4) {
		ROS_ERROR("Usage: centering <tolerance> <speed> <moveIncrement>");
		ROS_INFO("Loading default values...");
	} else {
		tolerance = atof(argv[1]);
		speed = atof(argv[2]);
		moveIncrement = atof(argv[3]);
	}
	ros::init(argc, argv, "centering");
	ros::NodeHandle n;
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	ros::Subscriber sub = n.subscribe("walls_topic", 1, center);
	//spin until centered
	/*
	while (ros::ok() && centeringPhase != 2) {
		ros::spinOnce();
	}
	*/
	turnAngle(1, 90);
	ROS_INFO("centering finished");
	return 0;

}
