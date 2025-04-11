/*
 * HTTPServer.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include <string>
#include <vector>
#include <mbed.h>
#include <EthernetInterface.h>
#include <SDBlockDevice.h>
#include <FATFileSystem.h>

class HTTPScript;

/**
 * The <code>HTTPServer</code> class implements a simple webserver that is able to
 * transmit files over an ethernet connection and allows to call scripts that are
 * registered with the server.
 * <br/>
 * An http server can be created and started as follows:
 * <pre><code>
 *   EthernetInterface* ethernet = new EthernetInterface();  <span style="color:#008000">// init the TCP/IP stack</span>
 *   ethernet->set_network("192.168.0.10", "255.255.255.0", "192.168.0.1");
 *   ethernet->connect();
 *
 *   HTTPServer* httpServer = new HTTPServer(ethernet); <span style="color:#008000">// creates an http server</span>
 *   ...
 * </code></pre>
 * This http server allows to execute application specific code implemented as http
 * scripts. These scripts are objects derived from the <code>HTTPScript</code> superclass.
 * <br/>
 * An example of an application specific script is given below:
 * <pre><code>
 * class MyHTTPScript : public HTTPScript {
 *     public:
 *                  MyHTTPScript();
 *         virtual  ~MyHTTPScript();
 *         string   call(vector<string> names, vector<string> values);
 * };
 * 
 * string MyHTTPScript::call(vector<string> names, vector<string> values) {
 *   
 *   string response;
 *   
 *   response += "  <h2>";
 *   for (uint32_t i = 0; i < min(names.size(), values.size()); i++) {
 *     response += "  <p>"+names[i]+"="+values[i]+"</p>";
 *   }
 *   response += "  </h2>";
 * 
 *   return response;
 * }
 * </code></pre>
 * This script returns the parameters that were passed to it by the http server.
 * <br/>
 * Before this script can be used, it needs to be registered with the http server
 * with the <code>add()</code> method as follows:
 * <pre><code>
 *   httpServer->add("myScript", new MyHTTPScript());
 * </code></pre>
 * When the <code>call()</code> method of the script is called by the http server,
 * it receives two string vectors: a vector with the names of the arguments passed
 * in the URL, and a vector with the corresponding values of the arguments.
 * <br/>
 * An example of an http request calling this script is as follows:
 * <pre><code>
 *   http://192.168.1.10/cgi-bin/myScript?x=0.5&y=-0.1&z=0.2
 * </code></pre>
 * The vectors of arguments passed to the <code>call()</code> method are then
 * {'x', 'y', 'z'} for the names and {'0.5', '-0.1', '0.2'} for the values.
 * <br/>
 * The response of the <code>call()</code> method is a <code>string</code> object
 * which is placed within an xhtml page, which in turn is returned by the http
 * server to the requesting http client.
 * @see HTTPScript
 */
class HTTPServer {
    
    public:
    
                    HTTPServer(EthernetInterface& ethernet);
        virtual     ~HTTPServer();
        void        add(std::string name, HTTPScript* httpScript);
        
    private:
        
        static const unsigned int   STACK_SIZE = 16384; // stack size of thread, given in [bytes]
        static const int            PORT_NUMBER = 80;   // port number of server to use
        static const unsigned int   INPUT_BUFFER_SIZE;  // size of receive buffer, given in [bytes]
        static const int            SOCKET_TIMEOUT;     // timeout of socket, given in [ms]
        
        EthernetInterface&          ethernet;
        TCPSocket                   server;
        SDBlockDevice*              sd;
        FATFileSystem*              fs;
        std::vector<std::string>    httpScriptNames;
        std::vector<HTTPScript*>    httpScripts;
        Thread                      thread;
        
        string      urlDecoder(std::string url);
        void        run();
};

#endif /* HTTP_SERVER_H_ */
