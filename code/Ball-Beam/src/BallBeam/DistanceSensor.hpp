#ifndef DISTANCE_SENSOR_HPP
#define DISTANCE_SENSOR_HPP

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

class DistanceSensor {
public:
    // Constructor with custom I2C, XSHUT pin, and loop period
    DistanceSensor(uint8_t xshut_pin, uint32_t loop_period_ms);
    
    // Destructor to clean up resources
    ~DistanceSensor();
    
    // Get distance in cm
    // Returns -1.0f if reading is invalid
    float get_distance();

    int getDiagnosticCode();
    void initialize();

private:
    const uint8_t pin_;             // XSHUT pin number
    VL53L4CD* sensor_;              // Pointer to sensor object
    bool is_initialized_;           // Flag to track initialization status
};

#endif // DISTANCE_SENSOR_HPP