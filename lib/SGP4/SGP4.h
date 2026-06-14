#ifndef SGP4_H
#define SGP4_H

#include <Arduino.h>

class Sgp4 {
public:
    Sgp4() {}
    
    void init(const char* tleLine1, const char* tleLine2) {
        // Mock implementation - does nothing
    }
    
    void findsat(int year, int month, int day, int hour, int minute, int second) {
        // Mock implementation - does nothing
    }
    
    void get_observe_navigator(double lat, double lon, double alt, 
                               double& azimuth, double& elevation, 
                               double& distance, double& altitude) {
        // Mock implementation - sets default values
        azimuth = 0.0;
        elevation = 0.0;
        distance = 0.0;
        altitude = 0.0;
    }
    
    void get_latlon(long double& lat, long double& lon) {
        // Mock implementation - sets default values
        lat = 0.0;
        lon = 0.0;
    }
};

#endif // SGP4_H
