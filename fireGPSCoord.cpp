// add needed libraries
#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>

// basic trig mapping algorithm, can upgrade for better accuracy 

const float EARTH_RADIUS = 6371000.0f;      // earth's radius 

// structure to hold Drone's GPS Coordinates (base reference point)
struct GPS {
    float latitude;
    float longitude;
};

// convert degrees to radians
float toRadians(float degrees) {
    return degrees * static_cast<float>(M_PI) / 180.0f;
}

// convert radians to degrees
float toDegrees(float radians) {
    return radians * 180.0f / static_cast<float>(M_PI);
}

// calculate fire GPS coordinates w given data
GPS fireCoordinates(float gimbalRotation, int altitude, GPS droneGPS, float lidarDistance) {
    float gimbalRadians = toRadians(gimbalRotation);
    // horizontal distance from drone to fire
    float horizontalDistance = lidarDistance * cos(gimbalRadians);
    // vertical distance from drone to ground
    float verticalDistance = lidarDistance * sin(gimbalRadians);

    // error check: fire is below or at drone's altitude 
    if (verticalDistance > altitude) {
        std::cerr << "Error: Fire detected above UAV. Check input values." << std::endl;
        return droneGPS;
    }

    // long and lat offsets
    float latitudeOffset = horizontalDistance / EARTH_RADIUS;
    float longitudeOffset = horizontalDistance / (EARTH_RADIUS * cos(toRadians(droneGPS.latitude)));

    // apply to drone's GPS coordinates
    GPS fireGPS;
    fireGPS.latitude = droneGPS.latitude + toDegrees(latitudeOffset);
    fireGPS.longitude = droneGPS.longitude + toDegrees(longitudeOffset);

    return fireGPS;
}

int main() {
    // testing values
    float gimbalRotation = 30.0f;               // gimbal rotation relative to gimbal's vertical axis
    int altitude = 120.0f;                      // drone's altitude above the ground from launch
    GPS droneGPS = {37.7749f, -122.4194f};      // drone coordinates
    float lidarDistance = 180.0f;               // LiDAR distance in meters

    GPS fireGPS = fireCoordinates(gimbalRotation, altitude, droneGPS, lidarDistance);

    // output
    std::cout << "Estimated Fire GPS Coordinates:\n";
    std::cout << "Latitude: " << fireGPS.latitude << "\n";
    std::cout << "Longitude: " << fireGPS.longitude << std::endl;

    return 0;
}
