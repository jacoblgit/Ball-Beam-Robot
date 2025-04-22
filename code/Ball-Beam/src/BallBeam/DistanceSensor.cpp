#include "DistanceSensor.hpp"

#define DEV_I2C Wire

DistanceSensor::DistanceSensor(uint8_t xshut_pin, uint32_t loop_period_ms)
    : pin_(xshut_pin), sensor_(nullptr), is_initialized_(false),
    timing_budget_(0.8 * loop_period_ms) {

    // note: timing budget in ms must be > 10ms and < 200ms
        
    sensor_ = new VL53L4CD(&DEV_I2C, xshut_pin);
}

void DistanceSensor::initialize() {
    // Check if sensor is already initialized
    if (is_initialized_) {
        return;
    }

    // Initialize I2C bus.
    DEV_I2C.begin();

    // Configure VL53L4CD satellite component.
    sensor_->begin();

    // Switch off VL53L4CD satellite component.
    sensor_->VL53L4CD_Off();

    //Initialize VL53L4CD satellite component.
    sensor_->InitSensor();

    // Program the highest possible TimingBudget, without enabling the
    // low power mode. This should give the best accuracy
    sensor_->VL53L4CD_SetRangeTiming(timing_budget_, 0);

    // Start Measurements
    sensor_->VL53L4CD_StartRanging();

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
        return -2.0f;
    }
    
    // Clear the interrupt to allow next measurement to occur
    sensor_->VL53L4CD_ClearInterrupt();
    
    // Get the measurement result
    if (sensor_->VL53L4CD_GetResult(&results) != 0) {
        return -3.0f;
    }
    
    // Check if the range status indicates a valid measurement
    // Range status of 0 means valid data
    if (results.range_status != 0) {
        return -4.0f;
    }
    
    return results.distance_mm;
}