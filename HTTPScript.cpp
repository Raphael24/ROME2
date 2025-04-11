/*
 * HTTPScript.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include "HTTPScript.h"

using namespace std;

HTTPScript::HTTPScript() {}

HTTPScript::~HTTPScript() {}

/**
 * This method must be implemented by derived classes.
 * It gets called by the http server, when an object of this class is
 * registered with the server, and the corresponding script is called
 * by an http client.
 * @param names a vector of the names of arguments passed to the server by
 * the client with a URL.
 * @param values a vector of the corresponding values of arguments passed
 * to the server.
 */
string HTTPScript::call(vector<string> names, vector<string> values) {
    
    return "";
}
