#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/StopMsg.h"
#include "red_fundamentals/StopSrv.h"
#include "create_fundamentals/DiffDrive.h"

bool should_stop = false;
bool should_pause = false;
double stop_dist = 0.15;
const int laser_msg_offset = 44;

void should_stop_check(){

	sensor_msgs::LaserScan::ConstPtr laser_message = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan_filtered", ros::Duration(1));

	if (!laser_message) {
		ROS_INFO("I couldn't find any laser message!");
		return; // return if "scan_filtered" didn't provide a message
	}

	// adjust for the first unusable values
	int valid_ranges_size = laser_message->ranges.size()-laser_msg_offset;

	// get the distance in front of the robot
	double dist = laser_message->ranges[valid_ranges_size/2+laser_msg_offset];
	// ROS_INFO("Wall at dist %f.\n", dist);
	if  (dist >= stop_dist || dist < laser_message->range_min) {
		should_stop = false;
	} else if (!isnan(dist)) {
		should_stop = true;
	}

	return;

}

bool execute_stop_callback(red_fundamentals::StopSrv::Request &req,

		red_fundamentals::StopSrv::Response &res){

	// set and reset for should_stop flag
	if (req.plead == "PAUSE") {
		should_pause = true;
		should_stop = false;
		res.processed = true;
		return true;
	}

	res.processed = false;
	return true;

}

int main(int argc, char **argv)
{

	if (argc != 4) {
		ROS_ERROR("Usage stopper <double stopDistance>");
		ROS_INFO("Setting default values...");
	} else {
		stop_dist = fabs(atof(argv[1]));
	}

	ros::init(argc, argv, "stopper");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("stop_service", execute_stop_callback);

	ros::Publisher pub = n.advertise<red_fundamentals::StopMsg>("stop_topic", 100);
	red_fundamentals::StopMsg msg;

	ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	create_fundamentals::DiffDrive ddsrv;

	while(ros::ok()) {

		ros::spinOnce();

		if (!should_pause){

			should_stop_check();

			if (should_stop) {
				ROS_INFO("Publishing STOP");
				msg.content = "STOP";
				pub.publish(msg);
			} else {
				ROS_INFO("Publishing FINE");
				msg.content = "FINE";
				pub.publish(msg);
			}

		} else {
			ROS_INFO("Publishing PAUSING");
			msg.content = "PAUSING";
			pub.publish(msg);
		}

	}

	ros::shutdown();
	return 0;

}
