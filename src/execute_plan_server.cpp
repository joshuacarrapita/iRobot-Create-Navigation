#include "ros/ros.h"
#include <cstdlib>
#include "red_fundamentals/ExecutePlan.h"
#include <vector>
#include "paradriver.hpp"
#include "moving.hpp"
#include "smooth_plan.hpp"
#include "red_fundamentals/Pose.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentals/WallsAlt.h"
#include "red_fundamentals/SimpleCall.h"
#include "song.hpp"

using namespace std;

ros::ServiceClient requestloc;
red_fundamentals::SimpleCall srv;
ros::Publisher pose_pub;
ros::Publisher request_pub;
red_fundamentals::Pose pose;
red_fundamentals::Grid map_cur;

bool walls[3] = {false,};
vector<int> wall_nums;
int row, column, orientation;
bool need_localization = false;

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
  	
  	/* else{
  		ROS_INFO("Waiting for the pose message from localization node...");
  	} */
  }
}

void mapCallback() {
    while(ros::ok()){
        const red_fundamentals::Grid::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::Grid>("map", ros::Duration(1.0));
        if(msg != nullptr){
            map_cur = *msg;
            ROS_INFO("Received map");
            break;
        }
        else{
            ROS_INFO("Waiting for the map information");
        }
    }
}

void wallCallback(){
    while(ros::ok()) {
        const red_fundamentals::WallsAlt::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::WallsAlt>("walls_alt_topic",
                                                                                                          ros::Duration(0.2));
        if(msg != nullptr) {
            wall_nums.clear();
            for (int i = 0; i < 3; ++i) {
                walls[i] = msg->walls[i];
                if (msg->walls[i]) {
                    wall_nums.push_back(i);
                }
            }
            break;
        }

        else{
            ROS_INFO("Waiting for the wall information...");
        }
    }
}

bool compareWalls(red_fundamentals::Cell cell, int orientation){
    if(wall_nums.size() != cell.walls.size()) return false;
    bool walls_robot[4] = {false,};
    
    for(int wall : cell.walls){
        // we convert wall direction in the map to one the robot is facing
        switch (orientation) {
            case 0:
                walls_robot[(wall + 1) % 4] = true;
                break;
            case 1:
                walls_robot[wall] = true;
                break;
            case 2:
                walls_robot[(wall + 3) % 4] = true;
                break;
            case 3:
                walls_robot[(wall + 2) % 4] = true;
                break;
            default:
                break;
        }
    }
    
    for(int i=0; i<3; i++){
        if(walls[i] != walls_robot[i]) return false;
    }
    return true;
}

void set_pose(int moved){
	switch(moved){
  		case RIGHT:
  			column += 1;
  			break;
  		case UP:
  			row -= 1;
  			break;
  		case LEFT:
  			column -= 1;
  			break;
  		case DOWN:
  			row += 1;
  			break;
  		default:
  			break;
  	}
  	orientation = moved;
	ROS_INFO("current position:");
	ROS_INFO("row: %d, column:%d, orientation:%d", row, column, orientation);
}

bool execute_plan(red_fundamentals::ExecutePlan::Request  &req,
          red_fundamentals::ExecutePlan::Response &res)
{
  
  // initialize variables 
  res.success = false;
  DiffDriving driver;
  Moving mover;
  Song s;
  // check if the plan is executable
  
  if(!is_executable(row, column, req.plan, map_cur)) {
	ROS_INFO("Plan not executable! Ready to run another one!");
  	return true;
  }

  for(size_t i = 0; i < req.plan.size(); i++){
  		switch((orientation - req.plan[i] + 4) % 4){
  			case 0:
  				break;
  			case 1: 
  				// do right 90° turn
				driver.turn(90);
  				break;
  			case 2:
  				// do 180° turn
  				driver.turn(180);
  				break;
  			case 3:
  				// do left 90° turn
				driver.turn(-90);
  				break;
  			default:
  				break;
  		}
  		
  		mover.moveDist(0.8);
  		set_pose(req.plan[i]);
  		
		wallCallback();	
  		if(!compareWalls(map_cur.rows[row].cells[column], orientation)){
  			s.playLostSound();
  			ROS_INFO("Lost! We have to relocalize ourselves...");
			need_localization = true;
  			return true;
  		}
  		
            	pose.row = row;
           	pose.column = column;
            	pose.orientation = orientation;
            	pose_pub.publish(pose);
            	ROS_INFO("Pose message published");
  	}
  
  driver.stop();
  res.success = true;
  ROS_INFO("Executed plan! Ready to run another one!");
  ROS_INFO("current pose:  row: %d, column: %d, orientation: %d", row, column, orientation);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smooth_server");
  ros::NodeHandle n;
  pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);
  
  mapCallback();
  ROS_INFO("Waiting for the pose message from localization node...");
  get_current_pose();
  requestloc = n.serviceClient<red_fundamentals::SimpleCall>("localization_service");
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan);
  ROS_INFO("Ready to execute plan");
  while(ros::ok()){
  	ros::spinOnce();
  	if(need_localization){
  	// TODO Call centering
  	ROS_INFO("Waiting for the pose message from localization node...");
	ros::Duration(3.0).sleep();
	requestloc.call(srv);
  	get_current_pose();
  	need_localization = false;
  	ROS_INFO("Ready to execute plan");
  	}
  }

  ros::spin();
  return 0;
}
