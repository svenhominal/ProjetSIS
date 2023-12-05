// File:          pioneer_interface.cpp
// Date:          31.07.2023
// Description:   Interface for the Pioneer robot (see: https://www.cyberbotics.com/doc/guide/pioneer-3at?version=cyberbotics:R2023a)
// Author:        Lucas Waelti

#pragma once

#include <math.h>
#include <string.h>
#include <string>
#include <random>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Receiver.hpp>

#define NUM_SENSORS   16  // Number of distance sensors
#define ROBOT_SLOWDOWN 4  // Slow down the robot by this factor

#define ACC_STD 0.05      // Standard deviation of the accelerometer noise
#define GYR_STD 0.025     // Standard deviation of the gyroscope noise

///////////////////////
// Robot information //
///////////////////////

/**
 * @brief      This structure provides useful information 
 *             about the Pioneer robot's dimensions.
*/
struct PioneerInfo{
  double length = 0.508;  // length of the robot [m]
  double width  = 0.497;  // width of the robot [m]
  double height = 0.277;  // height of the robot [m]
  double weight = 12.000; // weight of the robot [kg]
  double wheel_radius = 0.11; // Radius of a wheel [m]
  double axis_length  = 0.4;  // Distance between front/back pair of wheels[m]
  double max_wheel_speed = 6.4; // Maximum wheel speed [rad/s]
} typedef PioneerInfo;

static PioneerInfo pioneer_info; // Information about the Pioneer robot's dimensions


/////////////////////
// Robot interface //
/////////////////////

/**
 * @brief      Interface for the Pioneer robot
*/
class Pioneer{

public:
  
  /**
   * @brief      Pioneer interface constructor
  */
  Pioneer(){

    robot_ = new webots::Robot();

    timestep_ = (int)robot_->getBasicTimeStep();

    // Get the wheels and set them to velocity control
    motors_.reserve(4);
    motors_[0] = robot_->getMotor("back left wheel");
    motors_[1] = robot_->getMotor("back right wheel");
    motors_[2] = robot_->getMotor("front left wheel");
    motors_[3] = robot_->getMotor("front right wheel");
    for(int i=0; i<4; i++) motors_[i]->setPosition(INFINITY);

    // Initialize the wheel encoders
    encoders_.reserve(4);
    encoders_[0] = robot_->getPositionSensor("back left wheel sensor");
    encoders_[1] = robot_->getPositionSensor("back right wheel sensor");
    encoders_[2] = robot_->getPositionSensor("front left wheel sensor");
    encoders_[3] = robot_->getPositionSensor("front right wheel sensor");
    for(int i=0; i<4; i++) encoders_[i]->enable(timestep_);

    // Initialize the distance sensors
    dist_sensors_.reserve(NUM_SENSORS);
    for(int i=0; i<NUM_SENSORS; i++){
      std::string name = "so" + std::to_string(i);
      dist_sensors_[i] = robot_->getDistanceSensor(name);
      dist_sensors_[i]->enable(timestep_);
    }

    // Initialize the light sensor
    light_sensor_ = robot_->getLightSensor("light sensor");
    light_sensor_->enable(timestep_);

    // Initialize the accelerometer
    accelerometer_ = robot_->getAccelerometer("accelerometer");
    accelerometer_->enable(timestep_);

    // Initialize the gyroscope
    gyroscope_ = robot_->getGyro("gyro");
    gyroscope_->enable(timestep_);

    // Initialize the receiver
    receiver_ = robot_->getReceiver("receiver");
    receiver_->enable(timestep_);

    // Initialize the sensor noise distributions 
    acc_dist_.param(std::normal_distribution<double>::param_type(0.0, ACC_STD));
    gyr_dist_.param(std::normal_distribution<double>::param_type(0.0, GYR_STD));
  }

  /**
   * @brief      Pioneer interface destructor
  */
  ~Pioneer(){
    delete robot_;
  }

  /**
   * @brief      Perform some basic initialization
  */
  void init(){
    this->set_motors_velocity(0,0);
  }

  /**
   * @brief      Get the current time
   * @return     double: current time of the simulation in seconds
  */
  double get_time(){
    return robot_->getTime();
  }

  /**
   * @brief      Get the time step of the simulation in milliseconds 
   * @return     int: time step of the simulation in milliseconds
  */
  int get_timestep(){
    return (int)robot_->getBasicTimeStep();
  }

  /**
   * @brief      Step the simulation and update all sensor values
   * @return     int: 0 if the simulation is not over, -1 if it is over
  */
  int step(){
    for(int i=0; i<ROBOT_SLOWDOWN; i++){
      if(robot_->step(timestep_) == -1) return -1;
    }
    return 0;
  }

  /**
   * @brief      Set the velocity of the motors. Positive values 
   *             are forward, negative values are backward.
   * @param[in]  left:  velocity of the left motors [rad/s]
   * @param[in]  right: velocity of the right motors [rad/s]
  */
  void set_motors_velocity(double left, double right){
    motors_[0]->setVelocity(left);
    motors_[1]->setVelocity(right);
    motors_[2]->setVelocity(left);
    motors_[3]->setVelocity(right);
  }

  /**
   * @brief      Get the rotation angle of the wheels.
   *             Note: this returns a reference to the internal array, thus do not modify. 
   * @return     double[2]: left and right wheel rotation [rad]
  */
  double* get_encoders(){
    static double* values = new double[2];
    values[0] = 0.5*(encoders_[0]->getValue() + encoders_[2]->getValue());
    values[1] = 0.5*(encoders_[1]->getValue() + encoders_[3]->getValue());
    return values;
  }

  /**
   * @brief      Get the proximity sensor values. 
   *             The lookup table is as follows (format is: [distance measured std_dev, ...]):
   * 
   *             [
   *               0 1024 0.01,
   *               5    0 0.01
   *             ]
   *             
   *             Check the webots documentation for more information on how lookup tables work:
   *             https://www.cyberbotics.com/doc/reference/distancesensor#lookup-table
   * 
   * @return     double[NUM_SENSORS]: array of the distance sensor values
  */
  double* get_proximity(){

    static double* values = new double[NUM_SENSORS];

    for(int i=0; i<NUM_SENSORS; i++) 
      values[i] = dist_sensors_[i]->getValue();
    return values;
  }

  /**
   * @brief      Get the light sensor value (total irradiance E [W/m²])
   * @return     double: light sensor value
  */
  double get_light_intensity(){
    return light_sensor_->getValue();
  }

  /**
   * @brief      Get the inertial measurement unit (IMU) values (acc: m/s², gyro: rad/s)
   * @return     double[6]: IMU values [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
  */
  double* get_imu(){
    static double imu[6];
    const double* acc = accelerometer_->getValues();
    const double* gyro = gyroscope_->getValues();
    for(int i=0; i<3; i++){
      imu[i]   = acc[i]  + acc_dist_(gen_);
      imu[i+3] = gyro[i] + gyr_dist_(gen_);
    }
    return imu;
  }

  /**
   * @brief      Get the length of the message queue
   * @return     int: length of the message queue
  */
  int serial_get_queue_length(){
    return receiver_->getQueueLength();
  }

  /**
   * @brief      Get the first message in queue
   * @return     const double*: first message in queue
  */
  const double* serial_read_msg(){
    return (double*)receiver_->getData();
  }

  /**
   * @brief      Move to the next message in queue
  */
  void serial_next_msg(){
    receiver_->nextPacket();
  }

  /**
   * @brief      Get the signal strength of the last message
   * @return     signal strength of the last message (strength = 1 / r²)
  */
  double serial_get_signal_strength(){
    return receiver_->getSignalStrength();
  }

private:

  webots::Robot* robot_;   // webots robot instance 
  int timestep_;          // time step of the simulation in ms 

  std::vector<webots::Motor*> motors_;

  std::vector<webots::PositionSensor*> encoders_;

  std::vector<webots::DistanceSensor*> dist_sensors_;

  webots::LightSensor* light_sensor_;

  webots::Accelerometer* accelerometer_;

  webots::Gyro* gyroscope_;

  webots::Receiver* receiver_;

  // Sensor noise distributions
  std::default_random_engine gen_;
  std::normal_distribution<double> acc_dist_;
  std::normal_distribution<double> gyr_dist_;
};