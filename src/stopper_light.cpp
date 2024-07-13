#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/StopMsg.h"

#DEFINE LARRSIZE 726
#DEFINE LARROFFSET 44
#DEFINE LARRCTR (LARRSIZE/2)
#DEFINE LARRTHIRDSTA (LARRSIZE/3)
#DEFINE LARRTHIRDEND ((LARRSIZE/3) * 2)


bool should_stop = false;
double stop_dist = 0.10;
double call_frequency = -1.0;

red_fundamentals::StopMsg stop_msg;

void stoppah(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
	double minDist = 10.0;
	// get minimum distance in front third
	for (int i = LARRTHIRDSTA; i < LARRTHIRDEND; i++) {
		if (laser_msg->ranges[i] < minDist) {
			minDist = laser_msg->ranges[i];
		}
	}
	ROS_INFO("Wall at dist %f.\n", dist);
	if  (minDist >= stop_dist || minDist < laser_msg->range_min) {
		stop_msg.content = "STOP";
		ROS_INFO("Publishing STOP");
		pub.publish(stop_msg);
	} else {
		stop_msg.content = "FINE";
		ROS_INFO("Publishing FINE");
		pub.publish(stop_msg);
	}
	return;
}

int main(int argc, char **argv)
{

	if (argc != 3) {
		ROS_ERROR("Usage stopper <double stopDistance> <double StopCallFrequency (Hz | < 0.0 = inf)>");
		ROS_INFO("Setting default values...");
	} else {
		stop_dist = atof(argv[1]);
		if (stop_dist < 0) stop_dist * -1;
		call_frequency = atof(argv[2]);
	}

	ros::init(argc, argv, "stopper");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, stoppah);
	ros::Publisher pub = n.advertise<red_fundamentals::StopMsg>("stop_topic", 1);

	while (ros::ok()) { ros::spin(); }
	ros::shutdown();
	return 0;

}
