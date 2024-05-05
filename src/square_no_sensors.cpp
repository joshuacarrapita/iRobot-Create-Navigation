#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive srv;

double r = 0.033;
double d = 0.26;

void stop(){
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);
}

void move1m(){
	srv.request.left = M_PI;
	srv.request.right = M_PI;
	diffDrive.call(srv);
	ros::Duration(1/(M_PI*r)).sleep();
	stop();
}

void rotate90(){
	srv.request.left = - M_PI;
	srv.request.right = M_PI;;
	diffDrive.call(srv);
	ros::Duration(d/(4.0*r)).sleep();
	stop();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "example");
	ros::NodeHandle n;
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

	for(int j=0; j<5; j++){                 
		for(int i=0; i<4; i++){
			move1m();
			rotate90();
		}
	}  

	ros::spin();
	return 0;
}
