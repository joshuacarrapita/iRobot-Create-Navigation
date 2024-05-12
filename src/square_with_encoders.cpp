#include "ros/ros.h"
#include <cstdlib>
#include <math.h>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "red_fundamentals/PDController.h"
#include "moving.hpp"


bool correction = true;

float curr_left = 0.0;
float curr_right = 0.0;

float offset_left = 0.0;
float offset_right = 0.0;

float left = 0.0;
float right = 0.0;

float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.01;

float integral = 0;

// wanted angle
const float SETPOINT = 90;

ros::Publisher pd_controller_pub;
red_fundamentals::PDController msg;

create_fundamentals::DiffDrive srv_drive;
ros::ServiceClient diffDrive;

float deg_to_rad(float degree) {
    return degree * M_PI / 180.0;
}

float rad_to_deg(float rad) {
    return rad*180/M_PI;
}

float calculate_step(float error, float prev_error, float time_step) {
	
	// Proportional term
	float P = Kp * error;

	// Integral term
	integral += error * time_step;
	float I = Ki * integral;

	// Derivative term
	float derivative = (error - prev_error) / time_step;
	float D = Kd * derivative;

	// Calculate control signal
	float controlSignal = P + I + D;

	return controlSignal;
}

void moveCallback(const create_fundamentals::SensorPacket::ConstPtr& msg){

	float max_deg_error = 3;

	curr_left = msg->encoderLeft * time_step;
	curr_right = msg->encoderRight * time_step;

	float error_l = SETPOINT - rad_to_deg(curr_left);
	float error_r = - SETPOINT - rad_to_deg(curr_right);

	ROS_INFO("curr_left: %f", curr_left);
    ROS_INFO("curr_right: %f", curr_right);

	ROS_INFO("Error left %f", error_l);
    ROS_INFO("Error right %f", error_r);

	if(!(abs(error_l) >= max_deg_error || abs(error_r) >= max_deg_error)){
		correction = false;
		return;
	}

	float speed = 1;

	float new_angle = 90.0;
    int direction = speed;

	// Links korrigieren
	if (abs(error_l) >= max_deg_error) {
		
		new_angle = calculate_step(error_l, prev_error_l, time_step);

		if (new_angle > 0) {
			direction = speed;
		} else {
			direction = -speed;
		}
		
		srv_drive.request.left = direction;
		srv_drive.request.right = 0;

		diffDrive.call(srv_drive);
		ROS_INFO("Moving %f degrees left.", (new_angle));
		ros::Duration(deg_to_rad(abs(new_angle))/speed).sleep();
		
		// Update previous error for next iteration
		prev_error_l = error_l;
	}

	// Rechts korrigieren
	if (abs(error_r) >= max_deg_error) {

		new_angle = calculate_step(error_r, prev_error_r, time_step);

		if (new_angle > 0) {
			direction = speed;
		} else {
			direction = -speed;
		}
		srv_drive.request.left = 0;
		srv_drive.request.right = direction;

		diffDrive.call(srv_drive);
		ROS_INFO("Moving %f degrees right.", (new_angle));
		ros::Duration(deg_to_rad(abs(new_angle))/speed).sleep();
		
		// Update previous error for next iteration
		prev_error_r = error_r;
	}

	time_step += 0.1;

	pd_controller_pub.publish(msg);
}

void pd_control(){
	float time_step = 0.1;

	curr_left = curr_left * time_step;
	curr_right = curr_right * time_step;

	float error_l = SETPOINT - rad_to_deg(curr_left);
    float error_r = - SETPOINT - rad_to_deg(curr_right);

	float prev_error_l = 0.0;
    float prev_error_r = 0.0;

    float max_deg_error = 3;

	float speed = 1;

	while (abs(error_l) >= max_deg_error || abs(error_r) >= max_deg_error){
       
		curr_left = curr_left * time_step;
		curr_right = curr_right * time_step;
        
		float new_angle = 90.0;
        int direction = speed;

        error_l = SETPOINT - rad_to_deg(curr_left);
        error_r = -SETPOINT - rad_to_deg(curr_right);
        ROS_INFO("Error left %f", error_l);
        ROS_INFO("Error right %f", error_r);

		// Links korrigieren
        if (abs(error_l) >= max_deg_error) {
            
			new_angle = calculate_step(error_l, prev_error_l, time_step);
            
			if (new_angle > 0) {
                direction = speed;
            } else {
                direction = -speed;
            }
            
			srv_drive.request.left = direction;
            srv_drive.request.right = 0;

            diffDrive.call(srv_drive);
            ROS_INFO("Moving %f degrees left.", (new_angle));
            ros::Duration(deg_to_rad(abs(new_angle))/speed).sleep();
            prev_error_l = error_l;
        }

		// Rechts korrigieren
        if (abs(error_r) >= max_deg_error) {

            new_angle = calculate_step(error_r, prev_error_r, time_step);

            if (new_angle > 0) {
                direction = speed;
            } else {
                direction = -speed;
            }
            srv_drive.request.left = 0;
            srv_drive.request.right = direction;

            diffDrive.call(srv_drive);
            ROS_INFO("Moving %f degrees right.", (new_angle));
            ros::Duration(deg_to_rad(abs(new_angle))/speed).sleep();
            prev_error_r = error_r;
        }

		time_step += 0.1;

        pd_controller_pub.publish(msg);
        ros::spinOnce();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_with_encoders");
    ros::NodeHandle n;

    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::Subscriber sub_move = n.subscribe("sensor_packet", 1, moveCallback);
    pd_controller_pub = n.advertise<red_fundamentals::PDController>("pd_controller", 10);

	Moving mover;
	
	for (int i=0; i < 4; i++){
		
		// 1 Meter geradeaus fahren
		mover.moveDist(1);
		
		// Stoppen
		mover.stop();

		// Drehen / Korrigieren
		while(ros::ok() && correction){
			ros::spinOnce();
		}
		
        // Drehen / Korrigieren
        //pd_control();

        // Stoppen
		mover.stop();
    }

    ros::spin();
    ros::shutdown();

    return 0;
}