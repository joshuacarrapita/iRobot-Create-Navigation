#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>

using namespace std;

bool correcting = true;

float offset_left = 0.0;
float offset_right = 0.0;

float curr_left = 0.0;
float curr_right = 0.0;

float links = 0.0;
float rechts = 0.0;

const float Kp = 1.1;
const float Kd = 1.1;

const float SETPOINT = 90;

create_fundamentals::DiffDrive srv_drive;
ros::ServiceClient diffDrive;


void moveCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{

    curr_left = msg->encoderLeft;
    curr_right = msg->encoderRight;

    links = curr_left - offset_left;
    rechts = curr_right - offset_right;

    cout << endl;
    cout << "Callback: " << endl;
    cout << "current left = " << curr_left << endl;
    cout << "current right = " << curr_right << endl;
    cout << endl;
    cout << "offset_left = " << offset_left << endl;
    cout << "offset_right = " << offset_right << endl;
    cout << endl;
    cout << "left = " << links << endl;
    cout << "right = " << rechts << endl;
    cout << endl;
}

float deg_to_rad(float degree) {
    return degree * M_PI / 180.0;
}

float rad_to_deg(float rad) {
    return rad*180/M_PI;
}

float calculate_step(float error, float prev_error) {
    float proportional = Kp * error;
    float derivative = Kd * (error - prev_error);
    
    float out = proportional + derivative;
    return out;
}


void pd_control(){
    cout << "Start PD Control"<< endl;
    cout << endl;
    float error_l = 99.9;
    float error_r = 99.9;
    float speed = 1;

    float prev_error_l = 0.0;
    float prev_error_r = 0.0;

    float max_deg_error = 2.4;


    while (abs(error_l) >= max_deg_error || abs(error_r) >= max_deg_error){
        float new_angle = 90.0;
        
        int direction = speed;

        error_l = SETPOINT - rad_to_deg(links);
        error_r = -SETPOINT - rad_to_deg(rechts);
        cout << endl;
        cout << "error_l = " << SETPOINT << " - " << rad_to_deg(links) << " = " << error_l << endl;
        cout << "error_r = " << -SETPOINT << " - " << rad_to_deg(rechts) << " = " << error_r << endl;

        if (abs(error_l) >= max_deg_error) {

            new_angle = calculate_step(error_l, prev_error_l);

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

        if (abs(error_r) >= max_deg_error) {

            new_angle = calculate_step(error_r, prev_error_r);

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

        cout << "Jetzt müsste Callback kommen" << endl;
        ros::spinOnce();
        
        cout << "After correction: " << endl;
        cout << endl;
        cout << "current left = " << curr_left << endl;
        cout << "current right = " << curr_right << endl;
        cout << endl;
        cout << "offset_left = " << offset_left << endl;
        cout << "offset_right = " << offset_right << endl;
        cout << endl;
        cout << "left = " << links << endl;
        cout << "right = " << rechts << endl;
        cout << endl;  
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "squ_w_enc_lightblue");
    ros::NodeHandle n;

    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::Subscriber sub_move = n.subscribe("sensor_packet", 1, moveCallback);
    
    for (int i=0; i < 4; i++){

        // 1 Meter geradeaus fahren

        srv_drive.request.left = 5;
        srv_drive.request.right = 5;
        diffDrive.call(srv_drive);
        ros::Duration(1.0 / (5*0.031)).sleep();

        // Stoppen
        srv_drive.request.left = 0;
        srv_drive.request.right = 0;
        diffDrive.call(srv_drive);
        ros::Duration(1).sleep();

        // Collect current data
        cout << "Jetzt müsste Callback kommen" << endl;
        ros::spinOnce();
    
        offset_left = curr_left;
        offset_right = curr_right;
        
        links = curr_left - offset_left;
        rechts = curr_right - offset_right;

        cout << "Before first correction: " << endl;
        cout << endl;
        cout << "current left = " << curr_left << endl;
        cout << "current right = " << curr_right << endl;
        cout << endl;
        cout << "offset_left = " << offset_left << endl;
        cout << "offset_right = " << offset_right << endl;
        cout << endl;
        cout << "left = " << links << endl;
        cout << "right = " << rechts << endl;
        
        pd_control();
        cout << "Jetzt müsste Callback kommen" << endl;
        ros::spinOnce();

        //Stoppen
        srv_drive.request.left = 0;
        srv_drive.request.right = 0;
        diffDrive.call(srv_drive);
        ros::Duration(1).sleep();
        cout << "Jetzt müsste Callback kommen" << endl;
        ros::spinOnce();
    }

    //ros::spin(); //commenting out ros::spin() does nothing
    //ros::shutdown(); //commenting out ros::shutdown() does nothing

    return 0;
}