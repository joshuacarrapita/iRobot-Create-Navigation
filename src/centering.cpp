#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "sensor_msgs/LaserScan.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

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
	ROS_INFO("%f", msg->);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "centering");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("wall_info", 1, center);
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	//ros::spin();
	return 0;

}
