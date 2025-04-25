/*
 * SensorFusion.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "SensorFusion.h"

using namespace std;

const float SensorFusion::PERIOD = 0.002f;                  // period of task, given in [s]
const float SensorFusion::M_PI = 3.14159265358979323846f;   // the mathematical constant PI

const float SensorFusion::S_Q_ALPHA = 0.000010f;            // standard deviation of process parameter (angle)
const float SensorFusion::S_Q_OMEGA = 0.010000f;            // standard deviation of process parameter (rotation)
const float SensorFusion::S_R_ALPHA = 0.00001f;            // standard deviation of angle measurement
const float SensorFusion::S_R_OMEGA = 0.000001f;            // standard deviation of gyro measurement

const float SensorFusion::LOWPASS_FILTER_FREQUENCY = 25.0f;  // frequency of the lowpass filter, given in [rad/s]
const float SensorFusion::HIGHPASS_FILTER_FREQUENCY = 2000.0f; // frequency of the highpass filter, given in [rad/s]

/**
 * Creates a SensorFusion object.
 * @param imu a reference to the IMU to use.
 */
SensorFusion::SensorFusion(IMU& imu) : imu(imu), thread(osPriorityHigh, STACK_SIZE) {
    
    // initialize local values
    
    tiltAngleA = 0.0f;
    tiltAngleG = 0.0f;
    tiltAngleK = 0.0f;
    tiltAngleC = 0.0f;
    
    // initialize parameters for kalman filter
    
    p11 = 0.0f;
    p12 = 0.0f;
    p21 = 0.0f;
    p22 = 0.0f;
    xAlpha = 0.0f;
    xOmega = 0.0f;
    
    // initialize parameters for complementary filter
    
    alphaAccFiltered = 0.0f;
    alphaGyro = 0.0f;
    alphaGyroFiltered = 0.0f;
    
    // start thread and timer interrupt
    
    thread.start(callback(this, &SensorFusion::run));
    ticker.attach(callback(this, &SensorFusion::sendThreadFlag), PERIOD);
}

/**
 * Deletes the SensorFusion object.
 */
SensorFusion::~SensorFusion() {
    
    ticker.detach();
}

/**
 * Reads the tilt angle around the x-axis, calculated from accelerometer readings.
 * @return the tilt angle, given in [rad].
 */
float SensorFusion::readTiltAngleA() {
    
    return tiltAngleA;
}

/**
 * Reads the tilt angle around the x-axis, integrated from gyro readings.
 * @return the tilt angle, given in [rad].
 */
float SensorFusion::readTiltAngleG() {
    
    return tiltAngleG;
}

/**
 * Reads the tilt angle around the x-axis, obtained with a Kalman filter.
 * @return the tilt angle, given in [rad].
 */
float SensorFusion::readTiltAngleK() {
    
    return tiltAngleK;
}

/**
 * Reads the tilt angle around the x-axis, obtained with a complementary filter.
 * @return the tilt angle, given in [rad].
 */
float SensorFusion::readTiltAngleC() {
    
    return tiltAngleC;
}

/**
 * This method is called by the ticker timer interrupt service routine.
 * It sends a flag to the thread to make it run again.
 */
void SensorFusion::sendThreadFlag() {
    
    thread.flags_set(threadFlag);
}

/**
 * This <code>run()</code> method contains an infinite loop with the run logic.
 */
void SensorFusion::run() {
    
    while (true) {
        
        // wait for the periodic thread flag
        
        ThisThread::flags_wait_any(threadFlag);
        
        // read acceleration and gyro
        
        float accelerationY = -imu.readAccelerationY();
        float accelerationZ = imu.readAccelerationZ();
        float gyroX = imu.readGyroX();
        
        // calculate tilt angle from acceleration sensors and from gyro
        
        tiltAngleA = atan2(accelerationY, accelerationZ);
        tiltAngleG += gyroX*PERIOD;
        
        // calculate prediction for sensor fusion with Kalman-filter
        
        float zAlpha = atan2(accelerationY, accelerationZ);
        float zOmega = gyroX;
        
        xAlpha = xAlpha+PERIOD*xOmega;
        
        float p11 = this->p11+this->p12*PERIOD+this->p21*PERIOD+this->p22*PERIOD*PERIOD+S_Q_ALPHA*S_Q_ALPHA;
        float p12 = this->p12+this->p22*PERIOD;
        float p21 = this->p21+this->p22*PERIOD;
        float p22 = this->p22+S_Q_OMEGA*S_Q_OMEGA;
        
        this->p11 = p11;
        this->p12 = p12;
        this->p21 = p21;
        this->p22 = p22;
        
        // calculate correction for sensor fusion with Kalman-filter
                
        float k11 = p11/(p11+S_R_ALPHA*S_R_ALPHA);
        float k22 = p22/(p22+S_R_OMEGA*S_R_OMEGA);
        
        xAlpha = xAlpha+k11*(zAlpha-xAlpha);
        xOmega = xOmega+k22*(zOmega-xOmega);
        
        p11 = this->p11*(1.0-this->p11/(this->p11+S_R_ALPHA*S_R_ALPHA));
        p12 = 0.0;
        p21 = 0.0;
        p22 = this->p22*(1.0-this->p22/(this->p22+S_R_ALPHA*S_R_ALPHA));
        
        this->p11 = p11;
        this->p12 = p12;
        this->p21 = p21;
        this->p22 = p22;
        
        // set tilt angle from Kalman filter
        
        tiltAngleK = xAlpha;
        
        // set tilt angle from complementary filter
        
        float sf = LOWPASS_FILTER_FREQUENCY*PERIOD/(1.0f+LOWPASS_FILTER_FREQUENCY*PERIOD);
        alphaAccFiltered = sf*atan2(accelerationY, accelerationZ)+(1.0f-sf)*alphaAccFiltered;
        float alphaGyroNew = alphaGyro+PERIOD*gyroX;
        alphaGyroFiltered = 1.0f/(1.0f+HIGHPASS_FILTER_FREQUENCY*PERIOD)*(alphaGyroFiltered+alphaGyroNew-alphaGyro);
        alphaGyro = alphaGyroNew;
        
        tiltAngleC = alphaAccFiltered+alphaGyroFiltered;
    }
}
