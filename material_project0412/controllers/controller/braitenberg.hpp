#pragma once

#include "pioneer_interface/pioneer_interface.hpp"
#define NB_SENSORS 16
#define TIME_STEP 8
#define RANGE 1024/2 
#define MAXIMUM_SPEED 6.4
#define MAX_SPEED_WEB 6.7



/*

Pioneer sensor layout:

                Front
                3   4
        1   2           5   6
    0                           7
Left                             Right
    15                          8
        14  13          10  9
                12  11
                 Back

Response: >5m -> 0, 0m -> 1024
*/

/**
 * @brief      This function implements the Braitenberg algorithm
 *              to control the robot's velocity.
 * @param      ps           Proximity sensor readings (NUM_SENSORS values)
 * @param      vel_left     The left velocity
 * @param      vel_right    The right velocity
*/
void braitenberg(double* ps, double &vel_left, double &vel_right){


    // TODO: implement your Braitenberg algorithm here

    //double coefficients[16][2] =
    //{{-0.05, 0.3},{-0.1,0.4},{-0.15,0.5},{-0.2, 0.6},{0.6,-0.2},{0.5,-0.15},{0.4, -0.10},{0.3,-0.05},{0.05,-0.05},
    //{-0.1, -0.1},{-0.1,-0.1},{-0.8,-0.8},{-0.8, -0.8},{-0.1,-0.1},{-0.1,-0.1},{-0.1,0.1} };

    //double coefficients[16][2] =
    //{{-0.8, 0.2},{-0.8,0.3},{0.0,0.4}, // 0 1 2
    //{0.5, 0.5},{0.5,-0.5}, // 3 4
    //{0.4,-0.0},{0.3, 0.1},{0.2,-0.2}, // 5 6 7
    //{0.1,-0.4},{-0.1, -0.1},{-0.1,-0.1}, // 8 9 10
    //{-0.7,-0.7},{-0.7, -0.7}, // 11 12
    //{-0.1,-0.1},{-0.1,-0.1},{-0.1,0.1} }; // 13 14 15 

    double coefficients[16][2] =
    {{-0.4, 0.4},{-0.4,0.6},{-0.3,0.9}, // 0 1 2
    {-0.2, 1.3},{1.4,-0.35}, // 3 4
    {1.3,-0.2},{1.1, -0.2},{0.8,0.7}, // 5 6 7
    {-0.4,0.2},{-0.2, -0.3},{-0.3,-0.4}, // 8 9 10
    {-0.4,-0.4},{-0.4, -0.4}, // 11 12
    {-0.2,-0.6},{-0.2,-0.4},{0,0} }; // 13 14 15 


    vel_left = 0;
    vel_right = 0;

    for (int i = 0; i<NB_SENSORS;i++){

        vel_left += coefficients[i][0]*(1-ps[i]/RANGE);
        vel_right += coefficients[i][1]*(1-ps[i]/RANGE);

    }


    if (fabs(vel_left) > MAXIMUM_SPEED){
        vel_left = MAXIMUM_SPEED;
    }

    if (fabs(vel_right) > MAXIMUM_SPEED){
        vel_right = MAXIMUM_SPEED;
    }

}
