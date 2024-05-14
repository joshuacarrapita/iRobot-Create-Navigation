#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"

using namespace std;

int streak;
const float Kp = 1.1;
const float Kd = 1.1;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  
  float dist = 0.8 * streak;
  cout << "Wir wollen " << streak << " Zelle(n) fahren, also " << dist << " m geradeaus." << endl;

  float l_act = msg->encoderLeft;
  float r_act = msg->encoderRight;
  
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

}

void pd_control(){
  

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pd_control");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  Moving mover;

  // während der Abstand noch nicht richtig ist:
  //  hole dir neue Werte (ros::spinOnce())
  //  berechne Error und die Geschwindigkeit
  //  Fahre den Roboter max(Geschwindigkeit, 10)


  ros::spin();
  return 0;
}