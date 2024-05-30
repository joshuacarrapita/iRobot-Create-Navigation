#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "moving.hpp"

const float Kp = 1.1;

float l_act;
float r_act;

float error_l;
float error_r;

float speed_l;
float speed_r;

bool calling = false;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  l_act = msg->encoderLeft;
  r_act = msg->encoderRight;
  ROS_INFO("Callback called: left: %f, right: %f", l_act, r_act);
  calling = true;

}

void pd_control(Moving mover, int streak){
  ROS_INFO("Start of pd control");
  
  float dist = 1 * streak;
  ROS_INFO("Wir wollen %d Zelle(n) fahren, also %f m geradeaus", streak, dist);

  while(!calling){
    ROS_INFO("No encoder values yet");
    ros::spinOnce();
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }
  
  float l_des = l_act + (dist * 32);
  float r_des = r_act + (dist * 32);

  ROS_INFO("Actual Encoder Values: left: %f, right: %f", l_act, r_act);
  ROS_INFO("Desired Encoder Values: left: %f, right: %f", l_des, r_des);

  while(ros::ok()){

    ros::spinOnce();
    ros::Duration(0.2).sleep();
    ros::spinOnce();

    ROS_INFO("Actual Encoder Values: left: %f, right: %f", l_act, r_act);

    ROS_INFO("Desired Encoder Values: left: %f, right: %f", l_des, r_des);

    error_l = l_des - l_act;
    error_r = r_des - r_act;

    ROS_INFO("Error: left: %f, right: %f", error_l, error_r);

    if(l_act >= l_des || r_act >= r_des){       // Goal is reached
      
      break;

    } else if(r_act != 0 || l_act != 0){        // p-Control: calculating speed
      
      speed_l = Kp * error_l;
      speed_r = Kp * error_r;
      mover.move(speed_l, speed_r);

    } else {                                    // no encoder values yet
      
      ROS_INFO("No encoder values yet");
    }
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

  pd_control(mover, 1);

  return 0;
}