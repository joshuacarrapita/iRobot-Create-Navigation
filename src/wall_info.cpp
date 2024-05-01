#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/WallInfo.h"

red_fundamentals::WallInfo winfo_msg;
const float angle_step = 0.3515669;
const int laser_msg_offset = 44;
const int laser_msg_size = 681;
const int laser_msg_center = 341;

/*std::tuple<float,float> polarToCartesian(float radius, float angle);

*/
// converts polar to cartesian coordinates
std::tuple<float,float> polarToCartesian(float radius, float angle) {
	float x = std::cos(angle)*radius;
	float y = std::sin(angle)*radius;
	return {x,y};
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {

	std::tuple<float,float> coordinates[laser_msg_size];

	for (int i = 0; i < laser_msg_size; i++) {

		int dist = laser_msg->ranges[i+laser_msg_offset];
		int i_angle_offset = i - laser_msg_center; // i = 0 -> ang = -120 deg
		int angle = i_angle_offset*angle_step;

		coordinates[i] = polarToCartesian(dist, angle);
		//coordinates[i] = {std::cos(i*angle_step)*laser_msg->ranges[i+44],
		//std::sin(i*angle_step)*laser_msg->ranges[i+44]};
		
		ROS_INFO("Converted polar (r,a) -> cartesian (x,y): (%f,%f) -> (%f,%f)\n", dist, angle, std::get<0>(coordinates[i]), std::get<1>(coordinates[i]));

		i++;
	}

	winfo_msg.dist_wall_front = 110;
	winfo_msg.dist_wall_right = 111;
	winfo_msg.dist_wall_back = 112;
	winfo_msg.dist_wall_left = 113;
	winfo_msg.angle_wall_front = 90;
	winfo_msg.angle_wall_right = 91;
	winfo_msg.angle_wall_back = 92;
	winfo_msg.angle_wall_left = 93;
	
	ROS_INFO("WallInfo msg: %f, %f, %f, %f, %f, %f, %f, %f\n", winfo_msg.dist_wall_front, winfo_msg.dist_wall_right, winfo_msg.dist_wall_back, winfo_msg.dist_wall_left, winfo_msg.angle_wall_front, winfo_msg.angle_wall_right, winfo_msg.angle_wall_back, winfo_msg.angle_wall_left);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wall_info_talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<red_fundamentals::WallInfo>("wall_info_topic", 100);
	ros::Subscriber sub = n.subscribe("scan_filtered", 100, laserCallback);

	ros::Rate loop_rate(0.1);
	while (ros::ok()){
		ros::spinOnce();

		pub.publish(winfo_msg);

		loop_rate.sleep();
	}

	return 0;
}
