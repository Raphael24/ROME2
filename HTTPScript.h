/*
 * HTTPScript.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef HTTP_SCRIPT_H_
#define HTTP_SCRIPT_H_

#include <string>
#include <vector>

/**
 * This is the abstract http script superclass that needs to be derived
 * by application specific http scripts.
 * @see HTTPServer
 */
class HTTPScript {
    
    public:
        
                            HTTPScript();
        virtual             ~HTTPScript();
        virtual std::string call(std::vector<std::string> names, std::vector<std::string> values);
};

#endif /* HTTP_SCRIPT_H_ */
