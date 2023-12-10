


#include <math.h>
#include <memory.h>

#include "odometry.hpp"

///////////////////
// Eigen library //     DO NOT MODIFY THIS PART
///////////////////

#include <Eigen/Dense>

#define DIM 3                                       // State dimension 

typedef Eigen::Matrix<double,DIM,DIM>   Mat;        // DIMxDIM matrix  
typedef Eigen::Matrix<double, -1, -1>   MatX;       // Arbitrary size matrix 
typedef Eigen::Matrix<double,DIM,  1>   Vec;        // DIMx1 column vector  
typedef Eigen::Matrix<double, -1,  1>   VecX;       // Arbitrary size column vector  

static const Mat I = MatX::Identity(DIM,DIM);       // DIMxDIM identity matrix  

//////////////////////////////////
// Kalman filter base functions //    DO NOT MODIFY THIS PART
//////////////////////////////////

// State vector mu (x,y,heading) to be updated by the Kalman filter functions
static Vec mu = Vec::Zero();
// State covariance sigma to be updated by the Kalman filter functions
static Mat sigma = Mat::Zero();

/**
 * @brief      Get the state dimension 
*/
int kal_get_dim(){
    return DIM;
}

/**
 * @brief      Copy the state vector into a 1D array
*/
void kal_get_state(double* state){
    for(int i=0; i<DIM; i++){
        state[i] = mu(i);
    }
}

/**
 * @brief      Copy the state covariance matrix into a 2D array
*/
void kal_get_state_covariance(double** cov){
    for(int i=0;i<sigma.rows();i++){
        for(int j=0; j<sigma.cols(); j++){
            cov[i][j] = sigma(i,j);
        }
    }
}

/**
 * @brief      Check if a matrix contains any NaN values 
*/
bool kal_check_nan(const MatX& m){
    for(int i=0;i<m.rows();i++){
        for(int j=0; j<m.cols(); j++){
            if(isnan(m(i,j))){
                printf("FATAL: matrix has NaN values, exiting...\n");
                return true;
            }
        }
    }
    return false;
}



///////////////////////////////////////////////////
// TODO: implement your Kalman filter here after //
///////////////////////////////////////////////////

void kalman_filter_imu(Pose * odo, double * imu, int step){              // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
   
    double vx= imu[0]*step;
    double vy= imu[1]* step;
    double wz= imu[5]*step;
    //Mat sigma = Mat::Zero();
    MatX sigmax(3,3);
    sigmax<< ACC_STD*ACC_STD, 0,0,
            0, ACC_STD*ACC_STD,0,
            0,0, GYR_STD*GYR_STD;
    MatX A(3,3);
    MatX Tt(3,3);
    MatX Bt(3,3);
    MatX Rt(3,3);
    A = Mat::Identity();
    Tt << cos(odo->theta), -sin(odo->theta),0,
            sin(odo->theta), cos(odo->theta), 0,
            0,0,1;


    Bt= Tt*A*step;
    Rt= Bt*sigmax* Bt.transpose();

    // Prediction
    mu = A*mu + Bt* Vec(vx, vy, wz); //OK
    sigma= A* sigma*A.transpose()+ Rt;
    MatX C = Mat::Identity();

    // Update step 
    MatX K = sigma * C.transpose() * (C * sigma * C.transpose() + sigmax).inverse();
    mu = mu + K * (Vec(odo->x, odo->y, odo->theta) - C * mu);
    sigma = (Mat::Identity(3,3) - K*C) *sigma;
    // printf("With Kalman 1, x = %f, y = %f, heading = %f\n", mu(0), mu(1), RAD2DEG(mu(2)));
}

void kalman_filter_odometry(Pose * odo, double * imu, int step, double speed, double omega){
       
    double vx= odo->speed;
    double vy= 0;
    double wz= odo->omega;
    //printf("%f\n", vx);
    double sigmav=0.05;
    double sigmaw=0.05; 
    MatX sigmax(3,3);
    sigmax<< sigmav*sigmav,  0,0,
            0,0 ,0,
            0,0, sigmaw*sigmaw;
    MatX A(3,3);
    MatX Tt(3,3);
    MatX Bt(3,3);
    MatX Rt(3,3);
    A = Mat::Identity();
    Tt << cos(odo->theta), -sin(odo->theta),0,
            sin(odo->theta), cos(odo->theta), 0,
            0,0,1;


    Bt= Tt*A*step;
    Rt= Bt*sigmax* Bt.transpose();

    // Prediction
    mu = A*mu + Bt* Vec(vx, vy, wz); //OK
    sigma= A* sigma*A.transpose()+ Rt;
    MatX C = Mat::Identity();

    // Update step 
    MatX K = sigma * C.transpose() * (C * sigma * C.transpose() + sigmax).inverse();
    mu = mu + K * (Vec(odo->x, odo->y, odo->theta) - C * mu);
    sigma = (Mat::Identity(3,3) - K*C) *sigma;
    // printf("With Kalman 2, x = %f, y = %f, heading = %f\n", mu(0), mu(1), RAD2DEG(mu(2)));

}

/*
void kalman_filter_odometry(Pose * odo, double speed_wx, double speed_wy, int step){              // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
   
    double vx= odo->x;
    double vy= speed_wy; // = 0 ?
    double wz= odo->theta;

    double sigma_v = 0.05; //m/s
    double sigma_w = 10*GYR_STD;
    //Mat sigma = Mat::Zero();
    MatX sigmax(3,3);
    sigmax<< sigma_v, 0,0,
            0, 0 ,0,
            0,0, (10*GYR_STD)*(10*GYR_STD);
    MatX A(3,3);
    MatX Tt(3,3);
    MatX Bt(3,3);
    MatX Rt(3,3);
    A = Mat::Identity();
    Tt << cos(odo->theta), -sin(odo->theta),0,
            sin(odo->theta), cos(odo->theta), 0,
            0,0,1;
    Bt= Tt*A*step;
    Rt= Bt*sigmax* Bt.transpose();
    mu = A*mu + Bt* Vec(vx, vy, wz);
    sigma= A* sigma*A.transpose()+ Rt;
    MatX C = Mat::Identity();

    // Correction de mu et sigma
    MatX K = sigma * C.transpose() * (C * sigma * C.transpose() + sigmax).inverse();
    mu = mu + K * (Vec(odo->x, odo->y, odo->theta) - C * mu);

    printf("With Kalman 1, x = %f, y = %f, heading = %f\n", mu(0), mu(1), RAD2DEG(mu(2)));
}


*/






