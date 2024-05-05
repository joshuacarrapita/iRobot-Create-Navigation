#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

bool going_straight, rotating;
int cnt=0;
float initLeft, initRight, curLeft, curRight; // in rad
float radius = 0.033; // in m
// float perimeter = 2 * M_PI * radius; 
float d = 0.26; // not accurate
  
void reset(){
  initLeft = curLeft;
  initRight = curRight;  
}

void stop(){
  srv.request.left = 0;
  srv.request.right = 0;
  diffDrive.call(srv);
  going_straight = false;
  rotating = false;
}

void move(){
	srv.request.left = M_PI;
  	srv.request.right =  M_PI;
  	diffDrive.call(srv);
  	going_straight = true;
}

void rotate(){
	srv.request.left = - M_PI;
  	srv.request.right = M_PI;
  	diffDrive.call(srv);
  	rotating = true;
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  if(cnt >= 4) return;
  
  curLeft = (float)msg->encoderLeft;
  curRight = (float)msg->encoderRight;
  
  if(!going_straight && !rotating){
  	reset();
  	move();
  }
  
  else{
  	if(going_straight){
  	// >= dist
  	// abs
  		if((curLeft - initLeft) * radius >= 1) {
  			stop();
  			reset();
  			rotate();	
  		}
  	}
  	else if(rotating){
  	// >= (M_PI * d * theta) / 360
  		if((curRight - initRight) * radius >= (M_PI * d)/4) {
  		stop();
  		reset();
  		cnt ++;
  		if(cnt<4) move();
  	}
  }
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "square_with_encoders");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/sensorPacket", 1, sensorCallback);
  diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  
  ros::spin();
  return 0;


}


