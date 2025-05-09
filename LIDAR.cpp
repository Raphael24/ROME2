/*
 * LIDAR.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "LIDAR.h"

using namespace std;

const float LIDAR::DISTANCE_THRESHOLD = 0.01f;  // threshold for measured distance, given in [m]
const float LIDAR::DEFAULT_DISTANCE = 10.0f;    // default distance > range of sensor, given in [m]
const float LIDAR::M_PI = 3.1415926535897932f;  // the mathematical constant PI
const float LIDAR::DISTANCES[] = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 1.702465271f, 1.699141254f, 1.69632544f, 1.692140952f, 1.689068974f, 1.68018005f, 1.676267878f, 1.666183663f, 1.671424841f, 1.66193261f, 1.655635528f, 1.653413439f, 1.653517463f, 1.657246512f, 1.655132925f, 1.650946698f, 1.65257254f, 1.66468045f, 1.23646674f, 1.236336928f, 1.251003597f, 1.353653575f, 1.322500662f, 1.304577326f, 1.299988461f, 1.314887448f, 1.320968206f, 1.320374568f, 1.251579003f, 1.235510016f, 1.233241663f, 1.243382483f, 1.314194811f, 1.318788838f, 1.438384163f, 1.419872177f, 1.368804223f, 1.347354445f, 1.342721118f, 1.354318279f, 1.366872708f, 1.369305298f, 1.383822604f, 1.508895291f, 1.493255504f, 1.475824515f, 1.435599178f, 1.445460826f, 1.462035909f, 1.654100964f, 1.644884494f, 1.707480307f, 1.701130213f, 1.660187941f, 1.634974006f, 1.61723344f, 1.620856564f, 1.798737613f, 1.779742116f, 1.77366344f, 1.77661504f, 1.777926039f, 1.920203375f, 1.935389367f, 2.291142292f, 2.328650253f, 2.363611643f, 2.448420103f, 2.487483266f, 2.57330313f, 2.545476969f, 2.040235771f, 2.028301999f, 2.014f, 1.98730823f, 1.972207393f, 1.955661781f, 1.944761168f, 1.923351242f, 1.909502815f, 1.903193369f, 1.875251983f, 1.874046424f, 1.857301806f, 1.845873235f, 1.837153505f, 1.817614371f, 1.803495495f, 1.796232168f, 1.784177401f, 1.781868963f, 1.775984797f, 1.764001134f, 1.761087448f, 1.753326267f, 1.75371748f, 1.745729933f, 1.742740658f, 1.737636613f, 1.741089889f, 1.735240617f, 1.735295076f, 1.728695462f, 1.720377865f, 1.657877257f, 1.727796574f, 1.734111011f, 1.729579429f, 1.736116356f, 1.743193908f, 1.745805545f, 1.750349965f, 1.750429662f, 1.754628166f, 1.760757223f, 1.766661541f, 1.766717012f, 1.776524697f, 1.161069335f, 1.148512081f, 1.14054373f, 1.135753494f, 1.134139321f, 1.147087617f, 1.168199041f, 1.17903223f, 1.18040205f, 1.183327934f, 1.147416228f, 1.210927331f, 1.217378331f, 1.203945597f, 1.227244067f, 1.237879235f, 0.547902364f, 0.544882556f, 0.548745843f, 0.548314691f, 0.553859188f, 0.558237405f, 0.562782374f, 0.572875205f, 0.577382023f, 0.587456381f, 0.593270596f, 0.595157122f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 1.826932128f, 1.764292776f, 1.757409742f, 1.728041956f, 1.704264064f, 1.688f, 1.679250428f, 1.66501051f, 1.644250589f, 1.634979205f, 1.626211548f, 1.608795823f, 1.589880499f, 1.58137788f, 1.575325998f, 1.560708173f, 1.550516043f, 1.54374253f, 1.532342651f, 1.520213472f, 1.512674783f, 1.507533416f, 1.497487229f, 1.494217186f, 1.48919072f, 1.232154617f, 1.358305194f, 1.313616382f, 1.260025397f, 1.133025154f, 1.117565658f, 1.100202254f, 1.100181803f, 1.169824773f, 1.168748476f, 1.164819729f, 1.220040983f, 1.203097669f, 1.200735191f, 1.189294329f, 1.186247866f, 1.195532517f, 1.229603188f, 1.472497538f, 1.474618595f, 1.479985473f, 1.480043918f, 1.484135439f, 1.490767923f, 1.496454476f, 1.501894803f, 1.511313667f, 1.516939682f, 1.525908582f, 1.535359893f, 1.544518695f, 1.549271119f, 1.555904881f, 1.575204114f, 1.580785248f, 0.959320593f, 0.920540059f, 0.905058009f, 0.900680298f, 0.898481497f, 0.902731965f, 0.918059911f, 1.718295085f, 1.721370675f, 1.735719159f, 1.763130455f, 1.775434595f, 1.817423726f, 1.836723441f, 1.847665825f, 1.884798663f, 1.897461462f, 1.83701742f, 1.811276898f, 1.759177649f, 1.740189932f, 1.702600364f, 1.687345845f, 1.637890411f, 1.604450373f, 1.589151031f, 1.55510289f, 1.544042098f, 1.516327801f, 1.502226681f, 1.479634076f, 1.483579792f, 1.518056982f, 1.568964308f, 1.602244675f, 1.663f, 1.700264685f, 1.771085543f, 1.812491379f, 1.888618543f, 1.937385093f, 2.032088827f, 2.089617429f, 2.205471605f, 2.275026373f, 2.345580738f, 2.499928999f, 2.581471286f, 2.784285366f, 2.885689519f, 2.858447306f, 2.850508025f, 2.832713364f, 2.828422882f, 2.812354352f, 2.808188206f, 2.789411407f, 2.789161343f, 2.77675368f, 2.765194568f, 1.967565247f, 1.958f, 2.397585661f, 2.75211991f, 2.743884108f, 2.743676366f, 2.746035688f, 2.735854528f, 10.0f, 10.0f, 10.0f, 2.76492206f, 2.761207888f, 2.762739582f, 2.766510618f, 2.788146338f, 2.786430153f, 2.801847426f, 2.811054073f, 2.691653024f, 2.664378352f, 2.401709391f, 2.204667775f, 2.12351713f, 2.141373625f, 2.14578121f, 2.165700349f, 2.171425799f, 2.185005721f, 2.197850768f, 2.21938122f, 2.229375025f, 2.23809964f, 2.265003532f, 2.644680132f, 2.54522003f, 2.527676008f, 2.480120158f, 2.52638279f, 2.386449455f, 2.36217802f, 2.291129198f, 2.094351451f, 2.007193314f, 2.009421807f, 2.047382719f, 2.035974951f, 1.865168089f, 1.820468346f, 1.800660157f, 1.80447721f, 1.836480329f, 1.730293906f, 1.678679541f, 1.66250203f, 1.677434052f, 1.719175675f, 1.720679226f, 1.622129465f, 1.618845576f, 1.634181141f, 10.0f, 10.0f, 10.0f, 10.0f};

/**
 * Creates a LIDAR object.
 * @param serial a reference to a serial interface to communicate with the laser scanner.
 */
LIDAR::LIDAR(UnbufferedSerial& serial) : serial(serial) {
    
    // initialize serial interface
    
    serial.baud(115200);
    serial.format(8, SerialBase::None, 1);
    
    // initialize local values
    
    headerCounter = 0;
    dataCounter = 0;
    
    for (unsigned short i = 0; i < 360; i++) distances[i] = DEFAULT_DISTANCE;
    
    simulation = true;
    
    // start serial interrupt
    
    serial.attach(callback(this, &LIDAR::receive), UnbufferedSerial::RxIrq);
    
    // start the continuous operation of the LIDAR
    
    char bytes[] = {START_FLAG, SCAN};
    serial.write(&bytes, 2);
}

/**
 * Stops the lidar and deletes this object.
 */
LIDAR::~LIDAR() {
    
    // stop the LIDAR
    
    char bytes[] = {START_FLAG, STOP};
    serial.write(&bytes, 2);
}

/**
 * Get a list of points of a full 360 degree scan.
 * @return a deque vector of 360 point objects.
 */
deque<Point> LIDAR::getScan() {
    
    deque<Point> scan;
    
    for (unsigned short i = 0; i < 360; i++) {
        
        if (simulation) {
            
            // use simulated distances, because LIDAR is not available
            
            scan.push_back(Point(DISTANCES[i]-0.002f*(rand()%10), (float)i*M_PI/180.0f));
            
        } else {
            
            // use latest measurements from actual LIDAR
            
            scan.push_back(Point(distances[i], (float)i*M_PI/180.0f));
        }
    }
    
    return scan;
}

/**
 * Get a list of points which are part of beacons.
 * @return a deque vector of points that are beacons.
 */
deque<Point> LIDAR::getBeacons() {
    
    // get a list of all points of a scan
    
    deque<Point> scan = getScan();
    
    // create a list of points of beacons
    
    deque<Point> beacons;
    
    // check the points of a scan for beacons
    
    for (unsigned short i = 0; i < scan.size(); i++) {
        
        bool beacon = true; // flag to check if this point is possibly a beacon
        int counter = 0;
        
        // check distance to other points
        
        for (unsigned short j = 0; beacon && (j < scan.size()); j++) {
            
            float distance = scan[i].manhattanDistance(scan[j]);
            if (distance < 0.1f) {
                
                counter++; // another point which may be part of this beacon
                
            } else if (distance < 0.5f) {
                
                beacon = false; // this point cannot be part of a beacon
            }
        }
        
        if (beacon && (counter > 1)) beacons.push_back(scan[i]);
    }
    
    return beacons;
}

/**
 * This method is called by the serial interrupt service routine.
 * It handles the reception of measurements from the LIDAR.
 */
void LIDAR::receive() {
    
    // read received characters while input buffer is full
    
    if (serial.readable()) {
        
        // read single character from serial interface
        
        char byte = 0;
        serial.read(&byte, 1);
        
        // add this character to the header or to the data buffer
        
        if (headerCounter < HEADER_SIZE) {
            headerCounter++;
        } else {
            if (dataCounter < DATA_SIZE) {
                data[dataCounter] = byte;
                dataCounter++;
            }
            if (dataCounter >= DATA_SIZE) {
                
                // data buffer is full, process measurement
                
                char quality = data[0] >> 2;
                short angle = 360-(((unsigned short)data[1] | ((unsigned short)data[2] << 8)) >> 1)/64;
                float distance = (float)((unsigned short)data[3] | ((unsigned short)data[4] << 8))/4000.0f;
                
                if ((quality < QUALITY_THRESHOLD) || (distance < DISTANCE_THRESHOLD)) distance = DEFAULT_DISTANCE;
                
                // store distance in [m] into array of full scan
                
                while (angle < 0) angle += 360;
                while (angle >= 360) angle -= 360;
                distances[angle] = distance;
                
                // reset data counter and simulation flag
                
                dataCounter = 0;
                simulation = false;
            }
        }
    }
}
