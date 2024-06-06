#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/ExecutePlan.h"
#include "red_fundamentals/StopMsg.h"
#include "smooth_plan.hpp"
#include "paradriver.hpp"

using namespace std;

#define DRIVESPEED (2 * M_PI)
#define CURVESPEED (1.5 * M_PI)
#define WHEELRADIUS 0.032
#define WHEELDISTANCE 0.26

ros::ServiceClient diffDrive;
create_fundamentals::DiffDrive ddsrv;
red_fundamentals::Grid map_cur;
const double Kp = 1;

double l_act;
double r_act;

double error_l;
double error_r;

double speed_l;
double speed_r;

const double tolerance = 1;

int row, column, orientation;

void mapCallback() {
    while(ros::ok()){
	    const red_fundamentals::Grid::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::Grid>("map",
                                                                                                    ros::Duration(1.0));

	    if(msg != nullptr){
            	map_cur = *msg;
            	// receivedMap = true;
        	ROS_INFO("Received map");
		break;
    	    }
    	else{
        	ROS_INFO("Waiting for the map information");
    	}
    }
}

void get_current_pose(){
  // get a message from pose once
  while(ros::ok()){
  	const red_fundamentals::Pose::ConstPtr& msg = ros::topic::waitForMessage<red_fundamentals::Pose>("/pose", ros::Duration(1.0));
  
  	if (msg != nullptr){
    		row = msg->row;
    		column = msg->column;
    		orientation = msg->orientation;
    		ROS_INFO("Pose message arrived");
    		ROS_INFO("Pose after localization");
    		ROS_INFO("row: %d, column: %d, orientation: %d", row, column, orientation);
    		break;
  	}
  	
  	else{
    		ROS_INFO("Waiting for the pose message from localization node");
  	}
  }
}

bool stop_check(){

	red_fundamentals::StopMsg::ConstPtr stop_message = ros::topic::waitForMessage<red_fundamentals::StopMsg>("stop_topic", ros::Duration(1));

	if (stop_message && stop_message->content == "STOP") return true;
	return false;

}

bool sleep_and_stop(double duration){

	ros::Time then = ros::Time::now() + ros::Duration(duration);
	while (ros::Time::now() <= then) {
		if (stop_check()) {
			ddsrv.request.left = 0;
			ddsrv.request.right = 0;
			diffDrive.call(ddsrv);
			return false;
		}
	}
	return true;

}

bool execute_plan(red_fundamentals::ExecutePlan::Request  &req,
		red_fundamentals::ExecutePlan::Response &res)
{
	DiffDriving driver;

	// initialize variables 
	res.success = false;
	
	if(!is_executable(row, column, req.plan, map_cur)) return true;
	
	double theta_tmp = -90 + 90 * orientation;
	if(orientation % 2 == 0) theta_tmp *= -1;
		
	driver.resetOdometer(400 + 800 * column, 800 * map_cur.rows.size() - (400 + 800 * row), theta_tmp);
	
	// use paradriver to execute a plan
	for(size_t i = 0; i < req.plan.size(); i++){
  		switch(req.plan[i]){
  			case RIGHT:
  				column += 1;
  				driver.drive(0, driver.xcord + 800, driver.ycord, 0, 1);
  				break;
  			case UP:
  				row -= 1;
  				driver.drive(0, driver.xcord, driver.ycord + 800, 0, 1);
  				break;
  			case LEFT:
  				column -= 1;
  				driver.drive(0, driver.xcord - 800, driver.ycord, 0, 1);
  				break;
  			case DOWN:
  				row += 1;
  				driver.drive(0, driver.xcord, driver.ycord - 800, 0, 1);
  				break;
  			default:
  				break;
  		}
  	}
  
	
	driver.stop(); // This is so it doesn't keep driving after the very last
	orientation = floor((int)((driver.theta + 405) / 90) % 4);	
	res.success = true;
	ROS_INFO("Executed smooth plan! Ready to run another one!");
	ROS_INFO("current pose:  row: %d, column: %d, orientation: %d", row, column, orientation);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "smooth_server");
	ros::NodeHandle n;
	mapCallback();
	get_current_pose();
	ros::ServiceServer service = n.advertiseService("execute_smooth_plan", execute_plan);
	diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	
	ROS_INFO("Ready to run smooth plan");
	ros::spin();

	return 0;
}

