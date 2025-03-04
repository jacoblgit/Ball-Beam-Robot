#include <DistanceSensor.hpp>

DistanceSensor::DistanceSensor(TwoWire* i2c_bus, uint8_t xshut_pin, uint32_t loop_period_ms)
    : pin_(xshut_pin), sensor_(nullptr), is_initialized_(false) {
    // Calculate timing budget as 80% of loop period
    uint32_t timing_budget = loop_period_ms * 0.8;
    
    // Ensure timing budget is at least 10ms (minimum for the sensor)
    if (timing_budget < 10) {
        timing_budget = 10;
    }
    
    // Using provided I2C and XSHUT pin
    sensor_ = new VL53L4CD(i2c_bus, xshut_pin);
    
    if (!sensor_) {
        return;
    }
    
    // Begin sensor communication
    if (!sensor_->begin()) {
        return;
    }
    
    // Switch off VL53L4CD to ensure clean state
    sensor_->VL53L4CD_Off();
    
    // Initialize sensor
    if (sensor_->InitSensor() != 0) {
        return;
    }
    
    // Set the timing budget and disable low power mode (0)
    if (sensor_->VL53L4CD_SetRangeTiming(timing_budget, 0) != 0) {
        return;
    }
    
    // Start ranging
    if (sensor_->VL53L4CD_StartRanging() != 0) {
        return;
    }
    
    // If we made it here, initialization was successful
    is_initialized_ = true;
}

DistanceSensor::~DistanceSensor() {
    if (sensor_) {
        delete sensor_;
        sensor_ = nullptr;
    }
}

float DistanceSensor::get_distance() {
    // Check if sensor is properly initialized
    if (!is_initialized_ || !sensor_) {
        return -1.0f;
    }
    
    uint8_t data_ready = 0;
    VL53L4CD_Result_t results;
    
    // Check if new measurement is ready
    if (sensor_->VL53L4CD_CheckForDataReady(&data_ready) != 0 || data_ready == 0) {
        return -1.0f;
    }
    
    // Clear the interrupt to allow next measurement to occur
    sensor_->VL53L4CD_ClearInterrupt();
    
    // Get the measurement result
    if (sensor_->VL53L4CD_GetResult(&results) != 0) {
        return -1.0f;
    }
    
    // Check if the range status indicates a valid measurement
    // Range status of 0 means valid data
    if (results.range_status != 0) {
        return -1.0f;
    }
    
    // Convert from mm to cm and return
    return results.distance_mm / 10.0f;
}