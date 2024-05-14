#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"

using namespace std;

float left_enc;
float right_enc;

int streak;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  
  float dist = 0.8 * streak;
  cout << "Wir wollen " << streak << " Zelle(n) fahren, also " << dist << " m geradeaus." << endl;

  float l_act = msg->encoderLeft;
  float r_act = msg->encoderRight;
  
  cout << "Gerade sind die Encoder: "<< endl;
  ROS_INFO("left encoder: %f, right encoder: %f", left_enc, right_enc);

  float l_des = l_act + (dist * 30,3);
  float r_des = r_act + (dist * 30,3);

  
 
}

void pd_control(int streak){
  



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pd_control");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  Moving mover;


  ros::spin();
  return 0;
}