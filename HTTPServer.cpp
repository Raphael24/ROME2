/*
 * HTTPServer.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <algorithm>
#include <functional>
#include <stdio.h>
#include <errno.h>
#include "HTTPScript.h"
#include "HTTPServer.h"

using namespace std;

inline string int2String(int i) {
    
    char buffer[32];
    sprintf(buffer, "%d", i);
    
    return string(buffer);
}

const unsigned int HTTPServer::INPUT_BUFFER_SIZE = 1024;    // size of receive buffer, given in [bytes]
const int HTTPServer::SOCKET_TIMEOUT = 1000;                // timeout of socket, given in [ms]

/**
 * Create and initialize an http server object.
 * @param ethernet a reference to the embedded TCP/IP stack to use.
 */
HTTPServer::HTTPServer(EthernetInterface& ethernet) : ethernet(ethernet), thread(osPriorityNormal, STACK_SIZE) {
    
    sd = new SDBlockDevice(PE_6, PE_5, PE_2, PE_4);
    fs = new FATFileSystem("fs", sd);
    
    // start thread
    
    thread.start(callback(this, &HTTPServer::run));
}

/**
 * Delete the http server object.
 */
HTTPServer::~HTTPServer() {
    
    delete fs;
    delete sd;
}

/**
 * Registers the given script with the http server.
 * This allows to call a method of this script object
 * through virtual cgi-bin requests from a remote system.
 */
void HTTPServer::add(string name, HTTPScript* httpScript) {
    
    httpScriptNames.push_back(name);
    httpScripts.push_back(httpScript);
}

/**
 * Decodes a given URL string into a standard text string.
 */
string HTTPServer::urlDecoder(string url) {
    
    size_t pos = 0;
    while ((pos = url.find("+")) != string::npos) url = url.substr(0, pos)+" "+url.substr(pos+1);
    while ((pos = url.find("%08")) != string::npos) url = url.substr(0, pos)+"\b"+url.substr(pos+3);
    while ((pos = url.find("%09")) != string::npos) url = url.substr(0, pos)+"\t"+url.substr(pos+3);
    while ((pos = url.find("%0A")) != string::npos) url = url.substr(0, pos)+"\n"+url.substr(pos+3);
    while ((pos = url.find("%0D")) != string::npos) url = url.substr(0, pos)+"\r"+url.substr(pos+3);
    while ((pos = url.find("%20")) != string::npos) url = url.substr(0, pos)+" "+url.substr(pos+3);
    while ((pos = url.find("%22")) != string::npos) url = url.substr(0, pos)+"\""+url.substr(pos+3);
    while ((pos = url.find("%23")) != string::npos) url = url.substr(0, pos)+"#"+url.substr(pos+3);
    while ((pos = url.find("%24")) != string::npos) url = url.substr(0, pos)+"$"+url.substr(pos+3);
    while ((pos = url.find("%25")) != string::npos) url = url.substr(0, pos)+"%"+url.substr(pos+3);
    while ((pos = url.find("%26")) != string::npos) url = url.substr(0, pos)+"&"+url.substr(pos+3);
    while ((pos = url.find("%2B")) != string::npos) url = url.substr(0, pos)+"+"+url.substr(pos+3);
    while ((pos = url.find("%2C")) != string::npos) url = url.substr(0, pos)+","+url.substr(pos+3);
    while ((pos = url.find("%2F")) != string::npos) url = url.substr(0, pos)+"/"+url.substr(pos+3);
    while ((pos = url.find("%3A")) != string::npos) url = url.substr(0, pos)+":"+url.substr(pos+3);
    while ((pos = url.find("%3B")) != string::npos) url = url.substr(0, pos)+";"+url.substr(pos+3);
    while ((pos = url.find("%3C")) != string::npos) url = url.substr(0, pos)+"<"+url.substr(pos+3);
    while ((pos = url.find("%3D")) != string::npos) url = url.substr(0, pos)+"="+url.substr(pos+3);
    while ((pos = url.find("%3E")) != string::npos) url = url.substr(0, pos)+">"+url.substr(pos+3);
    while ((pos = url.find("%3F")) != string::npos) url = url.substr(0, pos)+"?"+url.substr(pos+3);
    while ((pos = url.find("%40")) != string::npos) url = url.substr(0, pos)+"@"+url.substr(pos+3);
    
    return url;
}

/**
 * This <code>run()</code> method binds the TCP/IP server to a given port number
 * and enters an infinite loop that waits for http requests and then processes
 * these requests and returns a response.
 */
void HTTPServer::run() {
    
    // bind the server to a given port number
    
    server.open(&ethernet);
    server.bind(PORT_NUMBER);
    server.listen();
    
    // enter infinite loop
    
    while (true) {
        
        TCPSocket* client = server.accept();
        if (client != NULL) {
            
            client->set_blocking(true);
            client->set_timeout(SOCKET_TIMEOUT); // set timeout of socket
            
            // read input
            
            char buffer[INPUT_BUFFER_SIZE];
            int size = client->recv(buffer, sizeof(buffer));
            
            if (size > 0) {
                
                string input(buffer, size);
                string header;
                string output;
                
                // parse input
                
                if ((input.find("GET") == 0) || (input.find("HEAD") == 0)) {
                    
                    if (input.find("cgi-bin") != string::npos) {
                        
                        // process script request with arguments
                        
                        string script = input.substr(input.find("cgi-bin/")+8, input.find(" ", input.find("cgi-bin/")+8)-input.find("cgi-bin/")-8);
                        string name;
                        vector<string> names;
                        vector<string> values;
                        
                        if (script.find("?") != string::npos) {
                            
                            name = script.substr(0, script.find("?"));
                            script = script.substr(script.find("?")+1);
                            
                            vector<string> arguments;
                            while (script.find("&") != string::npos) {
                                arguments.push_back(script.substr(0, script.find("&")));
                                script = script.substr(script.find("&")+1);
                            }
                            arguments.push_back(script);
                            
                            for (int i = 0; i < arguments.size(); i++) {
                                
                                if (arguments[i].find("=") != string::npos) {
                                    
                                    names.push_back(arguments[i].substr(0, arguments[i].find("=")));
                                    values.push_back(urlDecoder(arguments[i].substr(arguments[i].find("=")+1)));
                                    
                                } else {
                                    
                                    names.push_back(arguments[i]);
                                    values.push_back("");
                                }
                            }
                            
                        } else {
                            
                            name = script;
                        }
                        
                        // look for corresponding script
                        
                        for (int i = 0; i < min(httpScriptNames.size(), httpScripts.size()); i++) {
                            
                            if (httpScriptNames[i].compare(name) == 0) {
                                
                                output  = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n";
                                output += "<!DOCTYPE html>\r\n";
                                output += "<html xmlns=\"http://www.w3.org/1999/xhtml\" xml:lang=\"en\" lang=\"en\">\r\n";
                                output += "<body>\r\n";
                                output += httpScripts[i]->call(names, values);
                                output += "</body>\r\n";
                                output += "</html>\r\n";
                                
                                header  = "HTTP/1.1 200 OK\r\n";
                                header += "Content-Length: "+int2String(output.size())+"\r\n";
                                header += "Content-Type: text/xml\r\n";
                                header += "Expires: 0\r\n";
                                header += "\r\n";
                                
                                output = header+output;
                            }
                        }
                        
                        // requested script was not found on this server
                        
                        if ((output).size() == 0) {
                            
                            output  = "<!DOCTYPE html>\r\n";
                            output += "<html lang=\"en\">\r\n";
                            output += "<head>\r\n";
                            output += "  <title>404 Not Found</title>\r\n";
                            output += "  <style type=\"text/css\">\r\n";
                            output += "    h2 {font-family:Helvetica,Arial,sans-serif; font-size: 24; color:#FFFFFF;}\r\n";
                            output += "    p {font-family:Helvetica,Arial,sans-serif; font-size: 14; color:#444444;}\r\n";
                            output += "  </style>\r\n";
                            output += "</head>\r\n";
                            output += "<body leftmargin=\"0\" topmargin=\"0\" marginwidth=\"0\" marginheight=\"0\">\r\n";
                            output += "  <table width=\"100%\" height=\"100%\" border=\"0\" frame=\"void\" cellspacing=\"0\" cellpadding=\"20\">\r\n";
                            output += "    <tr>\r\n";
                            output += "      <th width=\"100%\" height=\"30\" bgcolor=\"#0064A6\"><h2>404 Not Found</h2></th>\r\n";
                            output += "    </tr>\r\n";
                            output += "    <tr>\r\n";
                            output += "      <td valign=\"top\">\r\n";
                            output += "      <p>The requested script could not be found on this server!</p>\r\n";
                            output += "      </td>\r\n";
                            output += "    </tr>\r\n";
                            output += "  </table>\r\n";
                            output += "</body>\r\n";
                            output += "</html>\r\n";
                            
                            header  = "HTTP/1.1 404 Not Found\r\n";
                            header += "Content-Length: "+int2String(output.size())+"\r\n";
                            header += "Content-Type: text/html\r\n";
                            header += "\r\n";
                            
                            output = header+output;
                        }
                        
                        // write output
                        
                        void* address = (void*)(output).c_str();
                        int offset = 0;
                        while (offset < (output).size()) offset += client->send((void*)(static_cast<int>(reinterpret_cast<intptr_t>(address))+offset), (output).size()-offset);
                        
                    } else {
                        
                        // look for file to load and transmit
                        
                        string filename = input.substr(input.find("/")+1, input.find(" ", input.find("/"))-input.find("/")-1);
                        if (filename.size() == 0) filename = "index.html";
                        filename = "/fs/"+filename;
                        
                        FILE* file = fopen(filename.c_str(), "r");
                        if (file != NULL) {
                            
                            // requested file exists
                            
                            fseek(file, 0, SEEK_END);
                            int32_t n = ftell(file);
                            
                            header  = "HTTP/1.1 200 OK\r\n";
                            header += "Content-Length: "+int2String(n)+"\r\n";
                            
                            if (filename.find(".htm") != string::npos) header += "Content-Type: text/html\r\n";
                            else if (filename.find(".txt") != string::npos) header += "Content-Type: text/plain\r\n";
                            else if (filename.find(".asc") != string::npos) header += "Content-Type: text/plain\r\n";
                            else if (filename.find(".css") != string::npos) header += "Content-Type: text/css\r\n";
                            else if (filename.find(".c") != string::npos) header += "Content-Type: text/plain\r\n";
                            else if (filename.find(".xml") != string::npos) header += "Content-Type: text/xml\r\n";
                            else if (filename.find(".dtd") != string::npos) header += "Content-Type: text/xml\r\n";
                            else if (filename.find(".js") != string::npos) header += "Content-Type: text/javascript\r\n";
                            else if (filename.find(".gif") != string::npos) header += "Content-Type: image/gif\r\n";
                            else if (filename.find(".jpg") != string::npos) header += "Content-Type: image/jpeg\r\n";
                            else if (filename.find(".png") != string::npos) header += "Content-Type: image/png\r\n";
                            else if (filename.find(".xbm") != string::npos) header += "Content-Type: image/x-xbitmap\r\n";
                            else if (filename.find(".xpm") != string::npos) header += "Content-Type: image/x-xpixmap\r\n";
                            else if (filename.find(".xwd") != string::npos) header += "Content-Type: image/x-xwindowdump\r\n";
                            else if (filename.find(".jar") != string::npos) header += "Content-Type: application/x-java-applet\r\n";
                            else if (filename.find(".pdf") != string::npos) header += "Content-Type: application/pdf\r\n";
                            else if (filename.find(".sig") != string::npos) header += "Content-Type: application/pgp-signature\r\n";
                            else if (filename.find(".spl") != string::npos) header += "Content-Type: application/futuresplash\r\n";
                            else if (filename.find(".ps") != string::npos) header += "Content-Type: application/postscript\r\n";
                            else if (filename.find(".dvi") != string::npos) header += "Content-Type: application/x-dvi\r\n";
                            else if (filename.find(".pac") != string::npos) header += "Content-Type: application/x-ns-proxy-autoconfig\r\n";
                            else if (filename.find(".swf") != string::npos) header += "Content-Type: application/x-shockwave-flash\r\n";
                            else if (filename.find(".tar.gz") != string::npos) header += "Content-Type: application/x-tgz\r\n";
                            else if (filename.find(".tar.bz2") != string::npos) header += "Content-Type: application/x-bzip-compressed-tar\r\n";
                            else if (filename.find(".gz") != string::npos) header += "Content-Type: application/x-gzip\r\n";
                            else if (filename.find(".tgz") != string::npos) header += "Content-Type: application/x-tgz\r\n";
                            else if (filename.find(".tar") != string::npos) header += "Content-Type: application/x-tar\r\n";
                            else if (filename.find(".bz2") != string::npos) header += "Content-Type: application/x-bzip\r\n";
                            else if (filename.find(".tbz") != string::npos) header += "Content-Type: application/x-bzip-compressed-tar\r\n";
                            else if (filename.find(".zip") != string::npos) header += "Content-Type: application/zip\r\n";
                            else if (filename.find(".mp3") != string::npos) header += "Content-Type: audio/mpeg\r\n";
                            else if (filename.find(".m3u") != string::npos) header += "Content-Type: audio/x-mpegurl\r\n";
                            else if (filename.find(".wma") != string::npos) header += "Content-Type: audio/x-ms-wma\r\n";
                            else if (filename.find(".wax") != string::npos) header += "Content-Type: audio/x-ms-wax\r\n";
                            else if (filename.find(".wav") != string::npos) header += "Content-Type: audio/x-wav\r\n";
                            else if (filename.find(".ogg") != string::npos) header += "Content-Type: audio/x-wav\r\n";
                            else if (filename.find(".mpg") != string::npos) header += "Content-Type: video/mpeg\r\n";
                            else if (filename.find(".mp4") != string::npos) header += "Content-Type: video/mp4\r\n";
                            else if (filename.find(".mov") != string::npos) header += "Content-Type: video/quicktime\r\n";
                            else if (filename.find(".qt") != string::npos) header += "Content-Type: video/quicktime\r\n";
                            else if (filename.find(".ogv") != string::npos) header += "Content-Type: video/ogg\r\n";
                            else if (filename.find(".avi") != string::npos) header += "Content-Type: video/x-msvideo\r\n";
                            else if (filename.find(".asf") != string::npos) header += "Content-Type: video/x-ms-asf\r\n";
                            else if (filename.find(".asx") != string::npos) header += "Content-Type: video/x-ms-asf\r\n";
                            else if (filename.find(".wmv") != string::npos) header += "Content-Type: video/x-ms-wmv\r\n";
                            
                            header += "\r\n";
                            
                            void* address = (void*)header.c_str();
                            int offset = 0;
                            while (offset < header.size()) offset += client->send((void*)(static_cast<int>(reinterpret_cast<intptr_t>(address))+offset), header.size()-offset);
                            
                            if (input.find("GET") == 0) {
                                
                                // transmit file
                                
                                fseek(file, 0, SEEK_SET);
                                uint8_t fileBuffer[1024];
                                int32_t read = 0;
                                while ((read = fread(fileBuffer, 1, 1024, file)) > 0) {
                                    void* address = (void*)fileBuffer;
                                    int offset = 0;
                                    while (offset < read) offset += client->send((void*)(static_cast<int>(reinterpret_cast<intptr_t>(address))+offset), read-offset);
                                }
                            }
                            
                            fclose(file);
                            
                        } else {
                            
                            // file not found
                            
                            output  = "<!DOCTYPE html>\r\n";
                            output += "<html lang=\"en\">\r\n";
                            output += "<head>\r\n";
                            output += "  <title>404 Not Found</title>\r\n";
                            output += "  <style type=\"text/css\">\r\n";
                            output += "    h2 {font-family:Helvetica,Arial,sans-serif; font-size: 24; color:#FFFFFF;}\r\n";
                            output += "    p {font-family:Helvetica,Arial,sans-serif; font-size: 14; color:#444444;}\r\n";
                            output += "  </style>\r\n";
                            output += "</head>\r\n";
                            output += "<body leftmargin=\"0\" topmargin=\"0\" marginwidth=\"0\" marginheight=\"0\">\r\n";
                            output += "  <table width=\"100%\" height=\"100%\" border=\"0\" frame=\"void\" cellspacing=\"0\" cellpadding=\"20\">\r\n";
                            output += "    <tr>\r\n";
                            output += "      <th width=\"100%\" height=\"30\" bgcolor=\"#0064A6\"><h2>404 Not Found</h2></th>\r\n";
                            output += "    </tr>\r\n";
                            output += "    <tr>\r\n";
                            output += "      <td valign=\"top\">\r\n";
                            output += "      <p>The requested file could not be found on this server!</p>\r\n";
                            output += "      </td>\r\n";
                            output += "    </tr>\r\n";
                            output += "  </table>\r\n";
                            output += "</body>\r\n";
                            output += "</html>\r\n";
                            
                            header  = "HTTP/1.1 404 Not Found\r\n";
                            header += "Content-Length: "+int2String(output.size())+"\r\n";
                            header += "Content-Type: text/html\r\n";
                            header += "\r\n";
                            
                            output = header+output;
                            
                            // write output
                            
                            void* address = (void*)output.c_str();
                            int offset = 0;
                            while (offset < output.size()) offset += client->send((void*)(static_cast<int>(reinterpret_cast<intptr_t>(address))+offset), output.size()-offset);
                        }
                    }
                    
                } else {
                    
                    // the http method is not known
                    
                    output  = "<!DOCTYPE html>\r\n";
                    output += "<html lang=\"en\">\r\n";
                    output += "<head>\r\n";
                    output += "  <title>400 Bad Request</title>\r\n";
                    output += "  <style type=\"text/css\">\r\n";
                    output += "    h2 {font-family:Helvetica,Arial,sans-serif; font-size: 24; color:#FFFFFF;}\r\n";
                    output += "    p {font-family:Helvetica,Arial,sans-serif; font-size: 14; color:#444444;}\r\n";
                    output += "  </style>\r\n";
                    output += "</head>\r\n";
                    output += "<body leftmargin=\"0\" topmargin=\"0\" marginwidth=\"0\" marginheight=\"0\">\r\n";
                    output += "  <table width=\"100%\" height=\"100%\" border=\"0\" frame=\"void\" cellspacing=\"0\" cellpadding=\"20\">\r\n";
                    output += "    <tr>\r\n";
                    output += "      <th width=\"100%\" height=\"30\" bgcolor=\"#0064A6\"><h2>400 Bad Request</h2></th>\r\n";
                    output += "    </tr>\r\n";
                    output += "    <tr>\r\n";
                    output += "      <td valign=\"top\">\r\n";
                    output += "      <p>The requested method is not supported by this server!</p>\r\n";
                    output += "      </td>\r\n";
                    output += "    </tr>\r\n";
                    output += "  </table>\r\n";
                    output += "</body>\r\n";
                    output += "</html>\r\n";
                    
                    header  = "HTTP/1.1 400 Bad Request\r\n";
                    header += "Content-Length: "+int2String(output.size())+"\r\n";
                    header += "Content-Type: text/html\r\n";
                    header += "\r\n";
                    
                    output = header+output;
                    
                    // write output
                    
                    void* address = (void*)output.c_str();
                    int offset = 0;
                    while (offset < output.size()) offset += client->send((void*)(static_cast<int>(reinterpret_cast<intptr_t>(address))+offset), output.size()-offset);
                }
            }
            
            client->close();
            
        } // client != NULL
        
    } // infinite while loop
}
