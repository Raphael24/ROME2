/*
 * LIDAR.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <cstdlib>
#include <deque>
#include <mbed.h>
#include "Point.h"

/**
 * This is a device driver class for the Slamtec RP LIDAR A1.
 */
class LIDAR {
    
    public:
        
                        LIDAR(UnbufferedSerial& serial);
        virtual         ~LIDAR();
        deque<Point>    getScan();
        deque<Point>    getBeacons();
        
    private:
        
        static const unsigned short HEADER_SIZE = 7;
        static const unsigned short DATA_SIZE = 5;
        
        static const char   START_FLAG = 0xA5;
        static const char   SCAN = 0x20;
        static const char   STOP = 0x25;
        static const char   RESET = 0x40;
        
        static const char   QUALITY_THRESHOLD = 10;     // quality threshold used for accepting measurements
        static const float  DISTANCE_THRESHOLD;         // threshold for measured distance, given in [m]
        static const float  DEFAULT_DISTANCE;           // default distance > range of sensor, given in [m]
        static const float  M_PI;                       // the mathematical constant PI
        static const float  DISTANCES[];                // simulated distance for every angle value, given in [m]
        
        UnbufferedSerial&   serial;             // reference to serial interface for communication
        char                headerCounter;
        char                dataCounter;
        char                data[DATA_SIZE];
        float               distances[360];     // measured distance for every angle value, given in [m]
        bool                simulation;         // flag to indicate if scans are only simulated
        
        void    receive();
};

#endif /* LIDAR_H_ */
