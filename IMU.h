/*
 * IMU.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef IMU_H_
#define IMU_H_

#include <cstdlib>
#include <mbed.h>
#include "ThreadFlag.h"
#include "LowpassFilter.h"

/**
 * This is a device driver class for the ST LSM9DS1 inertial measurement unit.
 */
class IMU {

    public:
        
                    IMU(SPI& spi, DigitalOut& csAG, DigitalOut& csM);
        virtual     ~IMU();
        float       readAccelerationX();
        float       readAccelerationY();
        float       readAccelerationZ();
        float       readGyroX();
        float       readGyroY();
        float       readGyroZ();
        float       readMagnetometerX();
        float       readMagnetometerY();
        float       readMagnetometerZ();
        float       readHeading();
        
    private:
        
        static const char   WHO_AM_I = 0x0F;
        static const char   CTRL_REG1_G = 0x10;
        static const char   CTRL_REG2_G = 0x11;
        static const char   CTRL_REG3_G = 0x12;
        static const char   OUT_X_L_G = 0x18;
        static const char   OUT_X_H_G = 0x19;
        static const char   OUT_Y_L_G = 0x1A;
        static const char   OUT_Y_H_G = 0x1B;
        static const char   OUT_Z_L_G = 0x1C;
        static const char   OUT_Z_H_G = 0x1D;
        static const char   CTRL_REG4 = 0x1E;
        static const char   CTRL_REG5_XL = 0x1F;
        static const char   CTRL_REG6_XL = 0x20;
        static const char   CTRL_REG7_XL = 0x21;
        static const char   CTRL_REG8 = 0x22;
        static const char   CTRL_REG9 = 0x23;
        static const char   CTRL_REG10 = 0x24;
        static const char   OUT_X_L_XL = 0x28;
        static const char   OUT_X_H_XL = 0x29;
        static const char   OUT_Y_L_XL = 0x2A;
        static const char   OUT_Y_H_XL = 0x2B;
        static const char   OUT_Z_L_XL = 0x2C;
        static const char   OUT_Z_H_XL = 0x2D;
        
        static const char   WHO_AM_I_M = 0x0F;
        static const char   CTRL_REG1_M = 0x20;
        static const char   CTRL_REG2_M = 0x21;
        static const char   CTRL_REG3_M = 0x22;
        static const char   CTRL_REG4_M = 0x23;
        static const char   CTRL_REG5_M = 0x24;
        static const char   OUT_X_L_M = 0x28;
        static const char   OUT_X_H_M = 0x29;
        static const char   OUT_Y_L_M = 0x2A;
        static const char   OUT_Y_H_M = 0x2B;
        static const char   OUT_Z_L_M = 0x2C;
        static const char   OUT_Z_H_M = 0x2D;
        
        static const unsigned int   STACK_SIZE = 2048;          // stack size of thread, given in [bytes]
        static const float          PERIOD;                     // period of task, given in [s]
        static const float          M_PI;                       // the mathematical constant PI
        static const float          LOWPASS_FILTER_FREQUENCY;   // frequency of the lowpass filter, given in [rad/s]
        
        SPI&            spi;
        DigitalOut&     csAG;
        DigitalOut&     csM;
        Mutex           mutex;
        ThreadFlag      threadFlag;
        Thread          thread;
        Ticker          ticker;
        
        float           magnetometerXMin;
        float           magnetometerXMax;
        float           magnetometerYMin;
        float           magnetometerYMax;
        LowpassFilter   magnetometerXFilter;
        LowpassFilter   magnetometerYFilter;
        float           heading;
        
        void    writeRegister(DigitalOut& cs, char address, char value);
        char    readRegister(DigitalOut& cs, char address);
        void    sendThreadFlag();
        void    run();
};

#endif /* IMU_H_ */
