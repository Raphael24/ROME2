/*
 * HTTPScriptSensorFusion.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef HTTP_SCRIPT_SENSOR_FUSION_H_
#define HTTP_SCRIPT_SENSOR_FUSION_H_

#include <string>
#include <vector>
#include "HTTPScript.h"
#include "SensorFusion.h"

/**
 * This is a specific http script to read the tilt angle from an imu.
 * @see HTTPServer
 */
class HTTPScriptSensorFusion : public HTTPScript {
    
    public:
        
                            HTTPScriptSensorFusion(SensorFusion& sensorFusion);
        virtual             ~HTTPScriptSensorFusion();
        virtual std::string call(std::vector<std::string> names, std::vector<std::string> values);
        
    private:
        
        SensorFusion&       sensorFusion;
};

#endif /* HTTP_SCRIPT_SENSOR_FUSION_H_ */
