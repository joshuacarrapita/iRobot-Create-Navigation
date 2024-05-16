#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"

float l_act;
float r_act;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  l_act = msg->encoderLeft;
  r_act = msg->encoderRight;
  ROS_INFO("Callback called: left: %f, right: %f", l_act, r_act);
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  Moving mover;

  while(ros::ok()){
    ROS_INFO("diffDrive 2 2");
    mover.move(2, 2);
    ros::spinOnce();
  }

  ROS_INFO("diffDrive 0 0");
  mover.move(0, 0);

  //ros::spinOnce();
  return 0;
}
