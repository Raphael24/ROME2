/*
 * HTTPScriptLIDAR.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef HTTP_SCRIPT_LIDAR_H_
#define HTTP_SCRIPT_LIDAR_H_

#include <string>
#include <vector>
#include "HTTPScript.h"
#include "LIDAR.h"

/**
 * This is a specific http script to read scans from a LIDAR.
 * @see HTTPServer
 */
class HTTPScriptLIDAR : public HTTPScript {
    
    public:
        
                            HTTPScriptLIDAR(LIDAR& lidar);
        virtual             ~HTTPScriptLIDAR();
        virtual std::string call(std::vector<std::string> names, std::vector<std::string> values);
        
    private:
        
        LIDAR&  lidar;
};

#endif /* HTTP_SCRIPT_LIDAR_H_ */
