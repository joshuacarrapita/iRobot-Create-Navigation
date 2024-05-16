#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

float l_act;
float r_act;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  l_act = msg->encoderLeft;
  r_act = msg->encoderRight;
  ROS_INFO("Callback called: left: %f, right: %f", l_act, r_act);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  
  create_fundamentals::DiffDrive srv;

  while(ros::ok()){
    ros::spinOnce();
    ROS_INFO("diffDrive 2 2");
    srv.request.left = 2;
    srv.request.right = 2;
    diffDrive.call(srv);
  }

  ROS_INFO("diffDrive 0 0");
  srv.request.left = 0;
  srv.request.right = 0;
  diffDrive.call(srv);

  ros::spinOnce();
  return 0;
}
