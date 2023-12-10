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

#define THRESHOLD   1024  // Was 1010 Arbitrary threshold value for the proximity sensors


//////////////////////
// Global variables //
//////////////////////



// enum used to store the current behavior
enum basicBehaviors {GOFORWARD, STOP, TURNING} behavior;

////////////////////////
// Behavior Functions //
////////////////////////
int turn_ctr = 0;
int ctr = 0; 
int Wall_end = 7;
int Wall_start = -1.3;
bool WALL_ODO = false;
bool WALL_SENSED = false;

void ForwardBrait(double* ps_values, double &vel_left, double &vel_right){

  braitenberg(ps_values, vel_left,vel_right);

  return;
}

void turningBehavior(double &vel_left, double &vel_right){
  vel_left = 0.27;
  vel_right = 1;
  return;
}

void ForwardProp(double* ps_values, double &vel_left, double &vel_right){

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
        //ForwardProp(ps_values, vel_left, vel_right);
        ForwardBrait(ps_values, vel_left, vel_right);

        // Detect wall with odometry
        WALL_ODO = pose.x>(Wall_end-1.4)||pose.x<0;

        // Detect wall with sensors
        WALL_SENSED = (ps_values[2]>800 && ps_values[3]>840 && ps_values[4]>840
        && ps_values[5]>800); 
        
        
        if(WALL_SENSED)behavior = TURNING; // rajouter WALL_ODO quand ODO = réglée
        
        // Check for transition criteria 
        for(int i=0; i<6; i++){ //was NUM_SENSORS 
          if (ps_values[i]>THRESHOLD){
            printf("Obstacle detected, stopping...\n");
            behavior = STOP;
          }
        }
        break;
      
      case TURNING:

        if(turn_ctr==0||turn_ctr==2 || turn_ctr==5){ // || turn_ctr == 3
    
          turningBehavior(vel_left, vel_right);
         
          //printf("left");
        }
        
        else if(turn_ctr==3){
          vel_left=0.0;
          vel_right=0.0;
          printf("One round finished.\n");
          behavior = STOP;

          return;
   
        }
        
        else{
          turningBehavior(vel_right, vel_left);
          //printf("right");
        }

 

        // printf("%d",ctr++); for debug

        // Check for transition criteria (depends on turn speed: 600 works well for 0.4 and 1.2):
        
        if(ctr++>660){ // rajouter aussi l'angle quand odométrie réglée
          ctr = 0;
          turn_ctr++;
          behavior = GOFORWARD;
          if(turn_ctr==6){
            turn_ctr=0;
          }
        }
        
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