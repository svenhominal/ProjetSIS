#pragma once 

#include "pioneer_interface/pioneer_interface.hpp"

#define PACKET_SIZE 5 // Number of doubles in a packet 

/**
 * @brief      Print an array of doubles in the terminal
 * @param[in]  array The array
*/
void print_array(double* array){
    printf("[");
    for(int i=0; i<PACKET_SIZE; i++){
        printf("%.3lf, ", array[i]);
    }
    printf("]\n");
}

double x=0; 
double y=0;
double id=0;
double tempin=0;
double tempout=0;
double  tempid=0;
double tempx=0;
double tempy=0;
double temptempin=0;
double temptempout=0;  
    

double previousstrength=0;
int i=0;
int readyB=1; 
double max=0;
int rangee=0;


double serial_get_data(Pioneer& robot, double* data,std::string f_temperature, int f_temperature_cols, Pose * pose, int turn_ctr, std::string f_data, int f_data_cols, double time){
    double strength=0.0 ; 

    int queueLength = robot.serial_get_queue_length();

    if (queueLength > 0) {


        const double* sensorData = robot.serial_read_msg();
        double strength= robot.serial_get_signal_strength();
        tempid=sensorData[0];
        log_csv(f_data, f_data_cols, sensorData[0], sensorData[1], sensorData[2], sensorData[3], sensorData[4], time);
        if (strength>max && int(tempid)==rangee){
              max=strength;
              tempid=sensorData[0];
              tempx=sensorData[1];
              tempy=sensorData[2];
              temptempin=sensorData[3];
              temptempout=sensorData[4];  
              readyB=1;
    
        }

        if (previousstrength>strength && readyB  && turn_ctr>=rangee && int(tempid) == turn_ctr) {
            log_csv(f_temperature, f_temperature_cols, tempid, tempx, tempy, temptempin, temptempout);  
            std::cout << "Sensor " << tempid << "," << tempx << "," <<tempy<< "," << temptempin<< "," << temptempout<< std::endl;
            readyB=0; 
            max=0; 
            rangee=rangee+1; 
        
        }
        
        
        previousstrength=strength; 

        robot.serial_next_msg();

    }

    
    
    return strength ; 
};



