#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/StopMsgLight.h"

#define LARRSIZE 726
#define LARROFFSET 44
#define LARRCTR (LARRSIZE/2)
#define LARRTHIRDSTA ((LARRSIZE/7) * 3)
#define LARRTHIRDEND ((LARRSIZE/7) * 4)


bool should_stop = false;
double stop_dist = 0.17;
double call_frequency = -1.0;

red_fundamentals::StopMsgLight stop_msg;
ros::Publisher pub;

void stoppah(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
	double minDist = 10.0;
	// get minimum distance in front third
	for (int i = LARRTHIRDSTA; i < LARRTHIRDEND; i++) {
		if (laser_msg->ranges[i] < minDist) {
			minDist = laser_msg->ranges[i];
		}
	}
	//ROS_INFO("Wall at dist %f.\n", minDist);
	if  (minDist >= stop_dist || minDist < laser_msg->range_min) {
		stop_msg.content = 0;
		//ROS_INFO("Publishing FINE");
		pub.publish(stop_msg);
	} else {
		stop_msg.content = 1;
		//ROS_INFO("Publishing STOP");
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
	pub = n.advertise<red_fundamentals::StopMsg>("stop_topic", 1);

	while (ros::ok()) { ros::spinOnce(); }
	ros::shutdown();
	return 0;

}
