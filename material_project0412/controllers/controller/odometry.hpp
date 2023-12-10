#pragma once 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"

#include <Eigen/Dense>

#define DIM 3                                       // State dimension 

typedef Eigen::Matrix<double,DIM,DIM>   Mat;        // DIMxDIM matrix  
typedef Eigen::Matrix<double, -1, -1>   MatX;       // Arbitrary size matrix 
typedef Eigen::Matrix<double,DIM,  1>   Vec;        // DIMx1 column vector  
typedef Eigen::Matrix<double, -1,  1>   VecX;       // Arbitrary size column vector  


#define RAD2DEG(X)      X / M_PI * 180.0 // Convert radians to degrees

// TODO: You can implement your wheel odometry here if relevant for your project


//Pose structure for the odometry with the wheel encoders
struct Pose {
    double x;
    double y;
    double theta;
    double speed;
    double omega;
};

// Pose structure for the odometry with the accelerometer
struct Pose_acc {
    double x_acc;
    double y_acc;
    double theta_acc;
};

//Set it to true to print the values of the odometry with wheel encoders 
#define VERBOSE_ODO_ENC true
#define VERBOSE_ODO_ACC false


double normalize_angle(double angle){
    while(angle>180) angle -= 360;
    while(angle <= -180) angle += 360;
    return angle;
} 


void odometry_accelerometer(Pose_acc* pose_acc, double Accx, double Accy, double gyro, double T, double time){
    
    double Acc[2] = {Accx, Accy};
    /*
    static double total_acc_x = 0;
    static double total_acc_y = 0;
    static int count = 0;
    static bool calibrate = true;
    static double start_time = 0;

    
    if (count == 0){
        start_time = time;
    }
    

    if (calibrate){
        total_acc_x += Acc[0];
        total_acc_y += Acc[1];
        count++;

        if(time-start_time>=4){
            calibrate = false;
        }
    } else {
        double mean_acc_x = total_acc_x/count;
        double mean_acc_y = total_acc_y/count;

        Acc[0] -= mean_acc_x;
        Acc[1] -= mean_acc_y;
    */

    
    double old_theta_acc = pose_acc->theta_acc ;
    pose_acc -> theta_acc  += RAD2DEG(gyro *T); //IN RADIANS
    pose_acc -> theta_acc = normalize_angle(pose_acc->theta_acc);

    static double speed[2] = {0,0};
        
    if (abs(old_theta_acc-pose_acc->theta_acc)>180){
        speed[0] = 0;
        speed[1] = 0;
    }

    MatX R(2,2);
    R<< cos(pose_acc->theta_acc), -sin(pose_acc->theta_acc),
        -sin(pose_acc->theta_acc), cos(pose_acc->theta_acc);

        
    double Acc_inertial[2] = {R(0,0)*Acc[0]+R(0,1)*Acc[1], R(1,0)*Acc[0] + R(1,1)*Acc[1]};

    speed[0] += Acc_inertial[0] *T;
    speed[1] += Acc_inertial[1]*T;
    

    pose_acc->x_acc += speed[0]*T;
    pose_acc->y_acc += speed[1]*T;


    if(VERBOSE_ODO_ACC){
        printf("Pose is : x = %f, y = %f, theta = %f\n", pose_acc->x_acc, pose_acc->y_acc, pose_acc->theta_acc);
    }
}
    
void odometry_encoders(Pose* odo, double Aleft_enc, double Aright_enc, double T) {
    
    //Add some noise (functions taken in the pioneer_interface)
    //std::default_random_engine generator;

    //Mean of 0 and a standard deviation of 0.1 (Asked in the Assignment to add some noise)
    //std::normal_distribution<double> distribution(0.0, 0.1); 

    //Previous values of encoders, initialisation here
    static double prev_left_enc = 0.0;
    static double prev_right_enc = 0.0;

    //Changes in encoder values 
    double delta_left_enc = Aleft_enc - prev_left_enc;
    double delta_right_enc = Aright_enc - prev_right_enc;

    
    
    //Add Gaussian noise 
    //delta_left_enc += distribution(generator);
    //delta_right_enc += distribution(generator);

    //Radians to meters 
    delta_left_enc = delta_left_enc * pioneer_info.wheel_radius;
    delta_right_enc = delta_right_enc * pioneer_info.wheel_radius;

    // Compute the forward and rotational speed, with the increments 
    odo-> speed = (delta_right_enc + delta_left_enc) / (2.0 * T);
    odo-> omega = (delta_right_enc - delta_left_enc) / (pioneer_info.axis_length * 2.94/2 * T); 

    // if (fabs(omega) < 0.01){}
      //*180/270;


	double speed_wx = odo->speed*cos(odo->theta);
	double speed_wy = odo->speed*sin(odo->theta);
	
	double omega_w  = odo->omega;
    odo->x += speed_wx* T;
    odo->y += speed_wy * T;
    odo->theta += (omega_w * T);

   
    // Update pose using the Euler method
    

    //Update the previous values for the computation of the next increment 
    prev_left_enc = Aleft_enc;
    prev_right_enc = Aright_enc;

    //while(odo->theta<0) odo->theta += 2*M_PI;
    //while(odo->theta>=2*M_PI) odo->theta -= 2*M_PI;


    //Print the pose values, if VERBOSE_ODO_ENC is true 
    if(VERBOSE_ODO_ENC){
        printf("Pose is : x = %f, y = %f, theta = %f, speed_forward = %f, omega = %f\n", odo->x, odo->y, RAD2DEG(odo->theta), odo->speed, odo->omega);
    }
}
