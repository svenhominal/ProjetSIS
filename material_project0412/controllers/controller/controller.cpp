// Controller for the robot robot

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

/* double sum = 0;
int j = 0;
int item = 0;
double max = 0;
double positionx[20000] = {0};
double positiony[20000] = {0};
double heading;

bool printedFirstLight = false;
bool printedSecondLight = false; 

 void getlight(const Pose& odo, double light){

  if (odo.x < 2){
    positionx[j] = odo.x;
    positiony[j] = odo.y;
    sum = sum + light;
    j++;
    if (light > max){
      max = light;
      item = j;

    }
    //printf("Light %.3f", odo.x);
  }
  else if (odo.x >=2 && odo.x < 4){
    if (!printedFirstLight){
      printf("Highest light intensity of first light  at x = %.3lf, y = %.3lf\n", positionx[item], positiony[item]);
      printedFirstLight = true;

      sum = 0;
      j = 0;
      item = 0;
      max = 0;
    }
    positionx[j] = odo.x;
    positiony[j] = odo.y; 
    sum = sum + light;
    j++;
    if (light > max){
      max = light;
      item = j;
    }
  }

  else {
    if (!printedSecondLight){
      printf("Highest light intensity of second light  at x = %.3lf, y = %.3lf\n", positionx[item], positiony[item]);
      printedSecondLight = true;

    }
  }
  
  return ;
}

*/

struct Light {
  double xStart;
  double xEnd;
  double maxIntensity;
  double posX;
  double posY;
  bool printed;
};


Light lights[3]{
  {0,2,0,0,0,false},
  {2,4,0,0,0,false},
  {4,6,0,0,0,false},
  };

void getlight(const Pose& pose, double light){
  for (int i = 0; i < 3; i++){
    if (pose.x >= lights[i].xStart && pose.x < lights[i].xEnd){
      if (light > lights[i].maxIntensity){
        lights[i].maxIntensity = light;
        lights[i].posX = pose.x;
        //printf("%3lf", lights[i].posX);
        lights[i].posY = pose.y;
      }
      //printf("Current odometry : x = %3lf, y = %3lf, heading = %.3lf\n", pose.x, pose.y, pose.theta);
    }
    if (pose.x >= lights[i].xEnd && !lights[i].printed){
      printf("Light %d detected ! State is XXX, at x = %3lf, y = %3lf \n", i+1, lights[i].posX, lights[i].posY);
      lights[i].printed = true;
    }
  }
  return;
}



int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer();
  robot.init();

  // Initialize an example log file
  std::string f_example = "example.csv";
  int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!

  //CSV for the pose with odometry 

  std:: string f_odometry = "odometry.csv";
  int         f_odometry_cols = init_csv(f_odometry, "time, x, y, heading");

  //Initial pose 
  Pose pose = {0,0,0};

  while (robot.step() != -1) {

    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////
    
    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity sensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity();  // Light intensity
    double* imu = robot.get_imu();                // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    ////////////////////
    // Implementation //
    ////////////////////


    //Compute odometry 
    

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // NAVIGATION
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds

    //FSM 
    fsm(ps_values, lws, rws, light, pose); // finite state machine 

    //Odometry
    computeOdometry(&pose, wheel_rot[0],wheel_rot[1],robot.get_timestep());

    //Light

    //Set velocity 

    getlight(pose, light);
    robot.set_motors_velocity(lws, rws); // set the wheel velocities


    //////////////////
    // Data logging //
    //////////////////

    // Log the time and light and IMU data in a csv file 
    log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);

    //Save data for odometry on a CSV file.
    log_csv(f_odometry, f_odometry_cols, time, pose.x, pose.y, pose.theta);

  }

  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}

