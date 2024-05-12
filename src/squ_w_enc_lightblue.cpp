#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <red_fundamentals/PDController.h>

using namespace std;


//create_fundamentals::ResetEncoders srv_reset;
//ros::ServiceClient reset;

float offset_left = 0.0;
float offset_right = 0.0;

float curr_left = 0.0;
float curr_right = 0.0;

float links = 0.0;
float rechts = 0.0;

const float Kp = 1.1;
const float Kd = 1.1;

const float SETPOINT = 90;

float t = 0.1;

ros::Publisher pd_controller_pub;
red_fundamentals::PDController msg;

create_fundamentals::DiffDrive srv_drive;
ros::ServiceClient diffDrive;


void moveCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  curr_left = msg->encoderLeft;
  curr_right = msg->encoderRight;
  links = curr_left - offset_left;
  rechts = curr_right - offset_right;
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

void turn(){
    float error_l = SETPOINT - rad_to_deg(links);
    float error_r = -SETPOINT - rad_to_deg(rechts);
    
    srv_drive.request.left = 2;
    srv_drive.request.right = 2;
    //ros::Duration(deg_to_rad(abs(new_angle))/speed).sleep();
    //ros::Duration(deg_to_rad(abs(new_angle))/speed).sleep();
    diffDrive.call(srv_drive);
}

void pd_control(){
    cout << "Start PD Control"<< endl;
    float error_l = 99.9;
    float error_r = 99.9;
    float speed = 1;
    //ros::Rate loop_rate(10);

    float prev_error_l = 0.0;
    float prev_error_r = 0.0;

    float max_deg_error = 3;


    while (abs(error_l) >= max_deg_error || abs(error_r) >= max_deg_error){
        float new_angle = 90.0;
        
        int direction = speed;
        msg.proportional_l;
        msg.proportional_r;
        msg.derivative_l;
        msg.derivative_r;

        error_l = SETPOINT - rad_to_deg(links);
        error_r = -SETPOINT - rad_to_deg(rechts);
        ROS_INFO("Error left %f", error_l);
        ROS_INFO("Error right %f", error_r);

        msg.error_l = error_l;
        msg.error_r = error_r;
    
        if (abs(error_l) >= max_deg_error) {
            msg.proportional_l = Kp * error_l;
            msg.derivative_l = Kd * (error_l-prev_error_l);
            new_angle = calculate_step(error_l, prev_error_l);
            msg.out_l = new_angle;
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
            msg.proportional_r = Kp * error_r;
            msg.derivative_r = Kd * (error_r-prev_error_r);
            new_angle = calculate_step(error_r, prev_error_r);
            msg.out_r = new_angle;
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

        pd_controller_pub.publish(msg);
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
        
    }
}


// void balance(){
//     float error = 0.0;
//     if (left < right) {
//         error = abs(right + left);
//     } else {
//         error = abs(left + right);
//     }

    // // Balance left
    // if (abs(left)-turn_rad < abs(right)-turn_rad){
    //     srv_drive.request.left = -1;
    //     srv_drive.request.right = 1;

    //     diffDrive.call(srv_drive);
    //     ros::Duration(error).sleep();
    //     ROS_INFO("correcting left by: %f", error);
    // } else {
    //     srv_drive.request.left = 1;
    //     srv_drive.request.right = -1;

    //     diffDrive.call(srv_drive);
    //     ros::Duration(error).sleep();
    //     ROS_INFO("correcting right by: %f", error);
    // }
    // ROS_INFO("end of balance");


    // while error not small enough
    // calculate p & d
    // move by new velocity
//}



int main(int argc, char **argv) {
    ros::init(argc, argv, "squ_w_enc_lightblue");
    ros::NodeHandle n;

    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::Subscriber sub_move = n.subscribe("sensorPacket", 1, moveCallback);
    pd_controller_pub = n.advertise<red_fundamentals::PDController>("pd_controller", 10);
/*
    srv_reset.response.success = true;
    reset.call(srv_reset);

    float encodeRefLeft = curr_left;
    float encodeRefRight = curr_right;

    for (int i = 0; i < 2; i++)
    {
        // 1 Meter geradeaus fahren
        srv_drive.request.left = 3;
        srv_drive.request.right = 3;
        diffDrive.call(srv_drive);
        //ros::Duration(1.0 / (5*0.031)).sleep();
        while(curr_left-encodeRefLeft < 29.8 && curr_right-encodeRefRight < 29.8){
            ros::spinOnce();
        }

        

        // Stoppen
        srv_drive.request.left = 0;
        srv_drive.request.right = 0;
        diffDrive.call(srv_drive);
        reset.call(srv_reset);
        encodeRefLeft = curr_left;
        encodeRefRight = curr_right;

        ros::Duration(1).sleep();

        // Collect current data
        ros::spinOnce();
        ROS_INFO("curr_left: %f", curr_left);
        ROS_INFO("curr_right: %f", curr_right);

        srv_drive.request.left = -3;
        srv_drive.request.right = 3;
        diffDrive.call(srv_drive);
        ros::Duration(M_PI / (2 * abs(srv_drive.request.right))).sleep();
        //while(curr_left-encodeRefLeft < 29.8 && curr_right-encodeRefRight < 29.8){
        //    ros::spinOnce();
        //}

        // Stoppen
        srv_drive.request.left = 0;
        srv_drive.request.right = 0;
        diffDrive.call(srv_drive);
        reset.call(srv_reset);
        encodeRefLeft = curr_left;
        encodeRefRight = curr_right;
        
        ros::Duration(1).sleep();

        // Collect current data
        ros::spinOnce();
        ROS_INFO("curr_left: %f", curr_left);
        ROS_INFO("curr_right: %f", curr_right);
    }
*/
    

    

    for (int i=0; i < 4; i++){

        // 1 Meter geradeaus fahren

        srv_drive.request.left = 5;
        srv_drive.request.right = 5;
        diffDrive.call(srv_drive);
        ros::Duration(1.0 / (5*0.031)).sleep();
        // while(curr_left-encodeRefLeft < 28.0 && curr_right-encodeRefRight < 28.0){
        //     ros::spinOnce();
        // }

        //encodeRefLeft = curr_left;
        //encodeRefRight = curr_right;

        // Stoppen
        srv_drive.request.left = 0;
        srv_drive.request.right = 0;
        diffDrive.call(srv_drive);
        ros::Duration(1).sleep();
        ros::spinOnce();


        // Collect current data
        ros::spinOnce();
        ROS_INFO("curr_left: %f", curr_left);
        ROS_INFO("curr_right: %f", curr_right);
    
        offset_left = curr_left;
        offset_right = curr_right;

        // Rotieren
        // srv_drive.request.left = -3;
        // srv_drive.request.right = 3;
        // diffDrive.call(srv_drive);
        // ros::Duration(SETPOINT).sleep();

        left = curr_left - offset_left;
        right = curr_right - offset_right;

        cout << "Before first correction: " << endl;
        cout << endl;
        cout << "current left = " << curr_left << endl;
        cout << "current right = " << curr_right << endl;
        cout << endl;
        cout << "offset_left = " << offset_left << endl;
        cout << "offset_right = " << offset_right << endl;
        cout << endl;
        cout << "left = " << left << endl;
        cout << "right = " << right << endl;

        //Stoppen
        pd_control();

        
        srv_drive.request.left = 0;
        srv_drive.request.right = 0;
        diffDrive.call(srv_drive);
        ros::Duration(1).sleep();
    }

    ros::spin();
    ros::shutdown();

    return 0;
}