#ifndef DISTANCE_SENSOR_HPP
#define DISTANCE_SENSOR_HPP

#include <Arduino.h>

class DistanceSensor {
public:
    // Constructor takes analog pin number
    explicit DistanceSensor(uint8_t pin);
    
    // Get distance in cm
    // Returns -1.0f if reading is invalid
    float get_distance();

private:
    const uint8_t pin_;
};

#endif // DISTANCE_SENSOR_HPP