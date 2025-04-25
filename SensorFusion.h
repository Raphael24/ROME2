/*
 * SensorFusion.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef SENSOR_FUSION_H_
#define SENSOR_FUSION_H_

#include <cstdlib>
#include <mbed.h>
#include "IMU.h"
#include "ThreadFlag.h"

/**
 * This class determines the IMU's tilt angle around the x-axis with sensor fusion algorithms.
 */
class SensorFusion {

    public:
        
                    SensorFusion(IMU& imu);
        virtual     ~SensorFusion();
        float       readTiltAngleA();
        float       readTiltAngleG();
        float       readTiltAngleK();
        float       readTiltAngleC();
        
    private:
        
        static const unsigned int   STACK_SIZE = 4096;          // stack size of thread, given in [bytes]
        static const float          PERIOD;                     // period of task, given in [s]
        static const float          M_PI;                       // the mathematical constant PI
        
        static const float          S_Q_ALPHA;
        static const float          S_Q_OMEGA;
        static const float          S_R_ALPHA;
        static const float          S_R_OMEGA;
        
        static const float          LOWPASS_FILTER_FREQUENCY;   // frequency of the lowpass filter, given in [rad/s]
        static const float          HIGHPASS_FILTER_FREQUENCY;  // frequency of the highpass filter, given in [rad/s]
        
        IMU&            imu;
        ThreadFlag      threadFlag;
        Thread          thread;
        Ticker          ticker;
        
        float   tiltAngleA;
        float   tiltAngleG;
        float   tiltAngleK;
        float   tiltAngleC;
        
        float   p11;
        float   p12;
        float   p21;
        float   p22;
        float   xAlpha;
        float   xOmega;
        
        float   alphaAccFiltered;
        float   alphaGyro;
        float   alphaGyroFiltered;
        
        void    sendThreadFlag();
        void    run();
};

#endif /* SENSOR_FUSION_H_ */
