// Provided libraries 
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"

// Files to implement your solutions  
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"

struct Light {
  double xStart;
  double xEnd;
  double maxIntensity;
  double posX;
  double posY;
 
  bool printed;
};

double moving_window[SIGNAL_LENGTH]={1};
double global_max = 2.1;
double local_max = 2.1;
double positionx = 0;
double positiony = 0;
double old_local_max = 2.1;
int light_condition = INIT;
int search_max = 1;
int time_step_ctr=0; 
int light_ctr =1;
bool light_found =  false;
bool ready = false;
int good_ctr=0;
int bad_ctr=0;

double omega;
double speed; 

int count_acc = 0;
double mean_acc_x = 0;
double mean_acc_y = 0;


int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer();
  robot.init();

  // Initialize an example log file
  /////int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!

  // CSV for the pose with odometry encoders
  std:: string f_odometry_enc = "odometry_encoders.csv";
  int         f_odometry_enc_cols = init_csv(f_odometry_enc, "time, x, y, heading,");

  // CSV for the pose with odometry accelerometer
  std:: string f_odometry_acc = "odometry_accelerometer.csv";
  int         f_odometry_acc_cols = init_csv(f_odometry_acc, "time, x, y, heading,");

  // CSV for the temperature
  std:: string f_temperature = "temperature.csv";
  int   f_temperature_cols = init_csv(f_temperature, "sensor id, x, int i=0;y, indoor temperature, outdoor temperature,");
  
  std:: string f_data= "data.csv";
  int f_data_cols= init_csv(f_data, "sensor id, x,y, indoor tempreature, outdoor temperature, time, "); 

  // Initial pose 
  Pose pose = {0,0,0,0,0};
  Pose_acc pose_acc= {0,0,0};

  // Delta for the time 
  double delta_time = 0;
  double previous_time = 0;

  
  //double acc_sum[3] = {0, 0, 0};
  //double acc_window[WINDOW_SIZE][3] = {0};
  //int count = 0;
  

  while (robot.step() != -1) {

    // Measurements acquisition //
    //////////////////////////////
    
    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity sensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity(); 
    double *imu= robot.get_imu();
    int steptime= robot.get_timestep();

    // Light intensity

    ////////////////////
    // Implementation //
    ////////////////////
 

    
    
    delta_time = time-previous_time;
    // DATA ACQUISITION
    double data[PACKET_SIZE];
    serial_get_data(robot, data, f_temperature, f_temperature_cols,  &pose,turn_ctr,  f_data, f_data_cols, time);
    
    //compute_moving_average(imu_values, acc_sum, acc_window, count);

    // NAVIGATION
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds

    // FSM 
    fsm(ps_values, lws, rws, light, pose); // finite state machine 

    // Odometry encoders
    odometry_encoders(&pose, wheel_rot[0],wheel_rot[1],delta_time);

    // Odometry accelerometer
    odometry_accelerometer(&pose_acc, imu[0], imu[1], imu[5], delta_time, robot.get_time());

    // Kalman filters 
    kalman_filter_odometry(&pose, imu,steptime, speed, omega);
    kalman_filter_imu(&pose, imu, steptime);

    // Update the time
    previous_time = time;

    // Light analysis
    light_analysis(light, moving_window,&pose);
 
    robot.set_motors_velocity(lws, rws); // set the wheel velocities


    //////////////////
    // Data logging //
    //////////////////

    // Log the time and light and IMU data in a csv file 
    //log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);

    //////////////////////////////void kalman_filter(pose * odo)
  
    //Save data for odometry on a CSV file.
    log_csv(f_odometry_enc, f_odometry_enc_cols, time, pose.x, pose.y, pose.theta);
    log_csv(f_odometry_acc, f_odometry_acc_cols, time, pose_acc.x_acc, pose_acc.y_acc, pose_acc.theta_acc);
   
  
  
  }
    
  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;

}



