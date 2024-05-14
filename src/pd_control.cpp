#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);

  ros::spinOnce();

  Moving mover;

  mover.moveDist(1);

  ros::spinOnce();
  
  return 0;
}