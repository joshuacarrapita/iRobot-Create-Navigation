#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"

using namespace std;

const float Kp = 1.1;

float l_act;
float r_act;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  cout << "Callback called" << endl;
  cout << endl;
  l_act = msg->encoderLeft;
  r_act = msg->encoderRight;

}

void pd_control(int streak){
  cout << "Start of pd control" << endl;
  cout << endl;
  
  float dist = 0.8 * streak;
  cout << "Wir wollen " << streak << " Zelle(n) fahren, also " << dist << " m geradeaus." << endl;
  
  cout << "Actual Encoder Values: "<< endl;
  ROS_INFO("left: %f, right: %f", l_act, r_act);

  float l_des = l_act + (dist * 30,3);
  float r_des = r_act + (dist * 30,3);

  cout << "Desired Encoder Values: "<< endl;
  ROS_INFO("left: %f, right: %f", l_des, r_des);

  float error_l = l_des - l_act;
  float error_r = r_des - r_act;

  cout << "Error: "<< endl;
  ROS_INFO("left: %f, right: %f", error_l, error_r);

  while(l_act < l_des || r_act < l_des){
    ros::spinOnce();

    cout << "Actual Encoder Values: "<< endl;
    ROS_INFO("left: %f, right: %f", l_act, r_act);

    l_des = l_act + (dist * 30,3);
    r_des = r_act + (dist * 30,3);

    cout << "Desired Encoder Values: "<< endl;
    ROS_INFO("left: %f, right: %f", l_des, r_des);

    error_l = l_des - l_act;
    error_r = r_des - r_act;

    cout << "Error: "<< endl;
    ROS_INFO("left: %f, right: %f", error_l, error_r);

    float speed_l = Kp * error_l;
    float speed_r = Kp * error_r;
    mover.move(speed_l, speed_r);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pd_control");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  Moving mover;

  ros::spinOnce();

  pd_control(1);


  ros::spin();
  return 0;
}