/*
 * HTTPScriptOrientation.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef HTTP_SCRIPT_ORIENTATION_H_
#define HTTP_SCRIPT_ORIENTATION_H_

#include <string>
#include <vector>
#include "HTTPScript.h"
#include "Controller.h"
#include "IMU.h"

/**
 * This is a specific http script to read the orientation from the controller and from the IMU.
 * @see HTTPServer
 */
class HTTPScriptOrientation : public HTTPScript {
    
    public:
        
                            HTTPScriptOrientation(Controller& controller, IMU& imu);
        virtual             ~HTTPScriptOrientation();
        virtual std::string call(std::vector<std::string> names, std::vector<std::string> values);
        
    private:
        
        Controller&     controller;
        IMU&            imu;
};

#endif /* HTTP_SCRIPT_ORIENTATION_H_ */
