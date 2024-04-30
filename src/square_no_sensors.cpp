#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

void stop(){
	  srv.request.left = 0;
	    srv.request.right = 0;
	      diffDrive.call(srv);
}

void move1m(){
       	for(int cnt=0; cnt<5; cnt++){
		srv.request.left = 2 * M_PI;
		srv.request.right = 2 * M_PI;
		diffDrive.call(srv);
		ros::Duration(1).sleep();
	}
	stop();
}

void rotate90(){
	srv.request.left = -2 * M_PI;
	srv.request.right = 2 * M_PI;;
	diffDrive.call(srv);
	ros::Duration(1).sleep();
	stop();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "example");
	ros::NodeHandle n;
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	                 
	for(int i=0; i<4; i++){
		move1m();
		rotate90();
	}
	  

	ros::spin();
	return 0;
}
