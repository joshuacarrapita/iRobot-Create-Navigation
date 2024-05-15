#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/StopMsg.h"
#include "red_fundamentals/Stopper.h"

bool should_stop = false;
bool should_reset = false;
double stop_dist = 0.15;

void should_stop_check(){
	sensor_msgs::LaserScan::ConstPtr laser_message = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan_filtered", ros::Duration(wait_wall_msg));

	if (!laser_message) {
		ROS_INFO("I couldn't find any laser message!");
		return; // return if "scan_filtered" didn't provide a message
	}

 	// adjust for the first unusable values
	int valid_ranges_size = laser_message->ranges.size()-laser_msg_offset;

	// get the distance in front of the robot
	double dist = laser_message->ranges[valid_ranges_size/2+laser_msg_offset]; 
	if (dist < laser_message->range_min) return; // no need to proceed if we didn't get a valid message

	if (dist < stop_dist) {
		should_stop = true;
		return;
	}

}

bool execute_stop_callback(red_fundamentals::StopSrv::Request &req,
          red_fundamentals::StopSrv::Response &res){

	// set and reset for should_stop flag
	if (req.plead == "STOP") {
		should_stop = true;
		res.processed = true;
		return true;
	}
	if (req.plead == "isweariwanttoresetandiwontbreakanything") {
		should_stop = false;
		should_reset = true;
		res.processed = true;
		return true;
	}

	return false;

}


int main(int argc, char **argv)
{

	if (argc != 2) {
		ROS_ERROR("Usage stop <stopDistance>");
		ROS_INFO("Setting default values...");
	} else {
		stop_dist = atof(argv[1]);
		if (stop_dist < 0) stop_dist * -1;
	}

	ros::init(argc, argv, "stopper");
	ros::NodeHandle n;
  	ros::ServiceServer service = n.advertiseService("stop", execute_stop_callback);
	ros::Publisher pub = n.advertise<red_fundamentals::StopMsg>("stop_topic", 100);
	red_fundamentals::StopMsg msg;

	while(ros::ok()) {

		ros::spinOnce();
		should_stop_check();
		if (should_stop) {
			msg.content = "STOP";
			pub.publish(msg);
		} 

 		// TODO: Maybe we don't need the reset function as other nodes would
		// read old messages first anyways
		if (should_reset) {
			msg.content = "CONTINUE";
			pub.publish(msg);
			should_reset = false;
		}

	}

	ros::shutdown()
	return 0;
}
