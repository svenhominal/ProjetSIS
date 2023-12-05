#pragma once 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"

#define RAD2DEG(X)      X / M_PI * 180.0 // Convert radians to degrees

// TODO: You can implement your wheel odometry here if relevant for your project


//Pose structure 
struct Pose {
    double x;
    double y;
    double theta;
};

//Set it to true to print the values of the odometry
#define VERBOSE_ODO_ENC false


void computeOdometry(Pose* odo, double Aleft_enc, double Aright_enc, double T) {
    
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
    delta_left_enc *= pioneer_info.wheel_radius;
    delta_right_enc *= pioneer_info.wheel_radius;

    // Compute the forward and rotational speed, with the increments 
    double speed = (delta_right_enc + delta_left_enc) / (2.0 * T);
    double omega = (delta_right_enc - delta_left_enc) / (pioneer_info.axis_length * T);

    // if (fabs(omega) < 0.01){}
    odo->x += speed * cos(odo->theta) * T;
    odo->y += speed * sin(odo->theta) * T;
    

    // Update pose using the Euler method
    
    odo->theta += omega * T;


    //Update the previous values for the computation of the next increment 
    prev_left_enc = Aleft_enc;
    prev_right_enc = Aright_enc;

    //while(odo->theta<0) odo->theta += 2*M_PI;
    //while(odo->theta>=2*M_PI) odo->theta -= 2*M_PI;


    //Print the pose values, if VERBOSE_ODO_ENC is true 
    if(VERBOSE_ODO_ENC){
        printf("Pose is : x = %f, y = %f, theta = %f\n", odo->x, odo->y, RAD2DEG(odo->theta));
    }
}
