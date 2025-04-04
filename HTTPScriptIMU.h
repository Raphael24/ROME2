/*
 * HTTPScriptIMU.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef HTTP_SCRIPT_IMU_H_
#define HTTP_SCRIPT_IMU_H_

#include <string>
#include <vector>
#include "HTTPScript.h"
#include "IMU.h"

/**
 * This is a specific http script to read sensor data from an imu.
 * @see HTTPServer
 */
class HTTPScriptIMU : public HTTPScript {
    
    public:
        
                            HTTPScriptIMU(IMU& imu);
        virtual             ~HTTPScriptIMU();
        virtual std::string call(std::vector<std::string> names, std::vector<std::string> values);
        
    private:
        
        IMU&    imu;
};

#endif /* HTTP_SCRIPT_IMU_H_ */
