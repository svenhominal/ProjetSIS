#pragma once 

/**
 * TODO: Implement your FSM here
 * 
 * This file implements a very basic FSM that initially lets the robot go forward
 * and stops it when an obstacle is detected.
 * 
 * You can modify this file to implement your own FSM and implement your own behaviors.
*/

#include "pioneer_interface/pioneer_interface.hpp" // contains the NUM_SENSORS constant 
#include "braitenberg.hpp" // you may want to use the braitenberg function to control the robot in some cases

#define THRESHOLD   5000  // Was 1010 Arbitrary threshold value for the proximity sensors


//////////////////////
// Global variables //
//////////////////////



// enum used to store the current behavior
enum basicBehaviors {GOFORWARD, STOP, TURNING, PROPORTIONAL} behavior;

////////////////////////
// Behavior Functions //
////////////////////////
int ctr = 0;
int lights_ctr = 0;
int ready = 1;
int light_duration = 0;
int Wall_end = 7;
int Wall_start = -1.3;
bool WALL_DETECTED = false;

void goForwardBehavior(double* ps_values, double &vel_left, double &vel_right){
  //vel_left = 1; 
  //vel_right = 1;
  braitenberg(ps_values, vel_left,vel_right);

/*

   if(ps_values[1]>950||ps_values[2]>950||ps_values[3]>900){
    vel_left = (ps_values[0]+2*ps_values[1]+3*ps_values[2]+4*ps_values[3])/4500;
    vel_right= 0;
  }
  else if(ps_values[6]>950||ps_values[5]>950||ps_values[4]>900){
    vel_right= (ps_values[7]+2*ps_values[6]+3*ps_values[5]+4*ps_values[4])/4500;
    vel_left=0;
  }
  else if (ps_values[7]==0||ps_values[8]==0){
    vel_left = 0.2;
    vel_right = 1;
  }
  else{
    vel_left=1;
    vel_right=1;
    //braitenberg(ps_values,vel_left,vel_right); //Braitenberg function
  }
  
  
  */

  return;
}

void turningBehavior(double &vel_left, double &vel_right){
  vel_left = 0.2;
  vel_right = 0.6;
  return;
}

void proportionalBehavior(double* ps_values, double &vel_left, double &vel_right){

   if(ps_values[1]>950||ps_values[2]>950||ps_values[3]>900){
    vel_left = (ps_values[0]+2*ps_values[1]+3*ps_values[2]+4*ps_values[3])/4500;
    vel_right= 0;
  }
  else if(ps_values[6]>950||ps_values[5]>950||ps_values[4]>900){
    vel_right= (ps_values[7]+2*ps_values[6]+3*ps_values[5]+4*ps_values[4])/4500;
    vel_left=0;
  }
  else if (ps_values[7]==0||ps_values[8]==0){
    vel_left = 0.2;
    vel_right = 1;
  }
  else{
    vel_left=1;
    vel_right=1;
  }


}


void stopBehavior(double &vel_left, double &vel_right){
  vel_left = 0; // was 0 
  vel_right = 0; // was 0 
  return;
}

///////////////////////
// Main FSM function //
///////////////////////

/**
 * @brief Finite State Machine that manages the robot's behavior
 * @param ps_values array of proximity values from the robot's proximity sensors
 * @param[out] vel_lef left wheels velocity
 * @param[out] vel_right right wheels velocity
*/
void fsm(double* ps_values, double &vel_left, double &vel_right, double light, Pose pose){
  

  switch(behavior){

      case GOFORWARD:
        
        // Perform the goForward behavior
        goForwardBehavior(ps_values, vel_left, vel_right);

        //TEST 
        if (light>2.1)
          //printf("%f \n", light);

        //Light detection 
        if (light>2100){
          light_duration++;
          printf("Detected light n%d, status : X, location  XXX", lights_ctr);

        }

        if(light_duration>40&&light<2100){
          light_duration = 0;
          lights_ctr++;
          printf("Detected light n%d, status : X, location  XXX", lights_ctr);
        }

        
        /*if (pose.x > 5 &&( ps_values[3]>500 || ps_values[4] > 500)){
          WALL_DETECTED = true;
          behavior = TURNING;
        }
        */
        if (pose.x > Wall_end-1.4){
          behavior = TURNING;
        }
        
        

        // Check for transition criteria 
        for(int i=0; i<6; i++){ //was NUM_SENSORS 
          if (ps_values[i]>THRESHOLD){
            printf("Obstacle detected, stopping...\n");
            behavior = STOP;
          }
        }
        break;
      
      case TURNING:
        turningBehavior(vel_left, vel_right);
        if (pose.theta >= 1.5*M_PI){
          behavior = GOFORWARD; //PROPORTIONAL 
        }
        break;

      case PROPORTIONAL:
        proportionalBehavior(ps_values, vel_left, vel_right);

        break;
        

      case STOP:

        // Perform the stop behavior
        stopBehavior(vel_left, vel_right);
        ctr++;

        // No transition criteria, the robot stays in this state forever
        break;

      default:
        printf("This behavior is not implemented.\n");
        break;
    }
    return;
}