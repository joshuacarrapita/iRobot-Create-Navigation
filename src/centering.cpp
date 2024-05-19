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
void turnRightAngle(int direction)
{
	stop();
	float ra_speed = 6.5;
	if (!direction) { ra_speed = -ra_speed; }
	srv.request.left = ra_speed;
	srv.request.right = -ra_speed;
	diffDrive.call(srv);
	ros::Duration(1.0).sleep();
	stop();
	return;
}

//turns specific angle between -90 and 90 degrees
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

//turns robot until it is roughly perpendicular to found wall
void perpendicularise(const red_fundamentals::Walls::ConstPtr& msg)
{
	//set variables if perpendicular
	if (msg->angle < 85 + tolerance*20 && msg->angle > 85 - tolerance*20) {
		ROS_INFO("already perpendicular...");
		isPerpendicular = 1;
		heading = 0;
		return;
	//turn right if wall right of robot
	} else if (msg->angle > 85 + tolerance*15) {
		ROS_INFO("turning right by %f degrees", (msg->angle - 86));
		turnAngle(1, msg->angle - 86);
		isPerpendicular = 1;
		heading = 0;
		return;
	//turn left if wall left of robot
	} else if (msg->angle < 88 - tolerance*15) {
		ROS_INFO("turning left by %f degrees", (88-msg->angle));
		turnAngle(0, 88 - msg->angle);
		isPerpendicular = 1;
		heading = 0;
		return;
	}
	return;
}

//moves robot until distance to wall is half cell width/length
void correctDist(const red_fundamentals::Walls::ConstPtr& msg)
{
	//if too far away move forward
	if (msg->dist > 0.24 + tolerance) {
		ROS_INFO("moving forward to correct distance");
		moveDist(1, msg->dist - 0.24);
		distCorrect = 1;
		if (centeringPhase == 1) {
			centeringPhase = 2;
		}
		return;
	//if too close move back
	} else if (msg->dist < 0.24 - tolerance) {
		ROS_INFO("moving backwards to correct distance");
		moveDist(-1, 0.24 - msg->dist);
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

//centers the robot in first available cell ends perpendicular to last found wall
void center(const red_fundamentals::Walls::ConstPtr& msg)
{
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
		//after finished adjusting robot to first wall turn right by 90° and look for wall in front
		if (heading == 0) {
			ROS_INFO("turning right to find second wall");
			turnRightAngle(1);
			heading = 1;
			ros::Duration(0.3).sleep();
			return;
		}
		//if found move on to phase 2
		if (heading == 1 && msg->dist < 0.8 && msg->angle > 75 && msg->angle < 105) {
			ROS_INFO("second wall found at heading 1");
			isPerpendicular = 0;
			distCorrect = 0;
			centeringPhase = 1;
			return;
		//if right of first wall no wall turn right by 180° and look for wall in front
		} else if (heading != 3) {
			ROS_INFO("turning 180 deg more to find second wall left of origin");
			turnRightAngle(0);
			turnRightAngle(0);
			heading = 3;
			ros::Duration(0.3).sleep();
			return;
		}
		//if found move on to phase 2
		if (msg->dist < 0.8 && heading == 3 && msg->angle > 75 && msg->angle < 105) {
			ROS_INFO("second wall found at heading 3 with dist: %f and angle %f", msg->dist, msg->angle);
			stop();
			isPerpendicular = 0;
			distCorrect = 0;
			centeringPhase = 1;
			return;
		} else {
			//if no suitable walls found directly connected to first wall, move forward with first wall left of robot and look for wall in front
			ROS_INFO("moving to find second wall");
			srv.request.left = 5;
			srv.request.right = 5;
			diffDrive.call(srv);
			//ros::Duration(moveIncrement).sleep();
			//stop();
			return;
		}
		//if found move on to phase 2
		if (msg->dist < 0.8 && msg->angle > 75 && msg->angle < 105) {
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
	ros::Duration(2.0).sleep();
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	ros::Subscriber sub = n.subscribe("walls_topic", 1, center);
	//spin until centered

	while (ros::ok() && centeringPhase != 2) {
		ros::spinOnce();
	}
	ROS_INFO("centering finished");
	return 0;

}
