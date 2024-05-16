#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"

const float Kp = 1.1;

float l_act;
float r_act;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  l_act = msg->encoderLeft;
  r_act = msg->encoderRight;
  ROS_INFO("Callback called: left: %f, right: %f", l_act, r_act);

}

void pd_control(Moving mover, int streak){
  ROS_INFO("Start of pd control");
  
  float dist = 0.8 * streak;
  ROS_INFO("Wir wollen %d Zelle(n) fahren, also %f m geradeaus", streak, dist);

  
  ROS_INFO("Wir fangen an zu fahren mit geringer Geschwindigkeit");
  mover.move(2, 2);

  ros::spinOnce();
  ROS_INFO("Actual Encoder Values: left: %f, right: %f", l_act, r_act);

  float l_des = l_act + (dist * 30,3);
  float r_des = r_act + (dist * 30,3);

  ROS_INFO("Desired Encoder Values: left: %f, right: %f", l_des, r_des);

  float error_l = l_des - l_act;
  float error_r = r_des - r_act;

  ROS_INFO("Error: left: %f, right: %f", error_l, error_r);

  while(ros::ok()){

    ros::spinOnce();

    if(l_act >= l_des || r_act >= r_des){
      break;
    }

    ROS_INFO("Actual Encoder Values: left: %f, right: %f", l_act, r_act);

    l_des = l_act + (dist * 30,3);
    r_des = r_act + (dist * 30,3);

    ROS_INFO("Desired Encoder Values: left: %f, right: %f", l_des, r_des);

    error_l = l_des - l_act;
    error_r = r_des - r_act;

    ROS_INFO("Error: left: %f, right: %f", error_l, error_r);

    ros::Duration(0.2).sleep();

    float speed_l = Kp * error_l;
    float speed_r = Kp * error_r;
    mover.move(speed_l, speed_r);

  }

  ROS_INFO("End of pd control! Stop");
  mover.move(0, 0);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pd_control");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  Moving mover;
  ros::spinOnce();

  pd_control(mover, 2);

  return 0;
}