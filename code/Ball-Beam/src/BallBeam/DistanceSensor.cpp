#include "DistanceSensor.hpp"

#define DEV_I2C Wire

DistanceSensor::DistanceSensor(uint8_t xshut_pin, uint32_t loop_period_ms)
    : pin_(xshut_pin), sensor_(nullptr), is_initialized_(false) {

    // VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);
        

    // // Initialize I2C bus.
    // DEV_I2C.begin();

    // // Configure VL53L4CD satellite component.
    // sensor_vl53l4cd_sat.begin();

    // // Switch off VL53L4CD satellite component.
    // sensor_vl53l4cd_sat.VL53L4CD_Off();

    // //Initialize VL53L4CD satellite component.
    // sensor_vl53l4cd_sat.InitSensor();

    // // Program the highest possible TimingBudget, without enabling the
    // // low power mode. This should give the best accuracy
    // sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);

    // // Start Measurements
    // sensor_vl53l4cd_sat.VL53L4CD_StartRanging();


    // Calculate timing budget as 80% of loop period
    // uint32_t timing_budget = loop_period_ms * 0.8;

    // // Ensure timing budget is at least 10ms (minimum for the sensor)
    // if (timing_budget < 10) {
    //     timing_budget = 10;
    // }

    sensor_ = new VL53L4CD(&DEV_I2C, xshut_pin);
    
    // i2c_bus->begin();
    // sensor_->begin();
    // sensor_->VL53L4CD_Off();
    // sensor_->InitSensor();
    // sensor_->VL53L4CD_SetRangeTiming(200,0);
    // sensor_->VL53L4CD_StartRanging();
    // is_initialized_ = true;

    // if (!sensor_) {
    //     return;
    // }
    
    // // Begin sensor communication
    // if (sensor_->begin() != VL53L4CD_ERROR_NONE) {
    //     return;
    // }
    
    // // Switch off VL53L4CD to ensure clean state
    // sensor_->VL53L4CD_Off();
    
    // // Initialize sensor
    // if (sensor_->InitSensor() != VL53L4CD_ERROR_NONE) {
    //     return;
    // }
    
    // // Set the timing budget and disable low power mode (0)
    // if (sensor_->VL53L4CD_SetRangeTiming(timing_budget, 0) != VL53L4CD_ERROR_NONE) {
    //     return;
    // }
    
    // // Start ranging
    // if (sensor_->VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE) {
    //     return;
    // }
    
    // // If we made it here, initialization was successful
    // is_initialized_ = true;
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
    sensor_->VL53L4CD_SetRangeTiming(200, 0);

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
    if (!is_initialized_) {
        return -1.0f;
    }

    if(!sensor_) {
        return -1.5f;
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
    
    // Convert from mm to cm and return
    return results.distance_mm / 10.0f;
}

int DistanceSensor::getDiagnosticCode() {
    if (!sensor_) return 1;  // Sensor object creation failed
    if (!is_initialized_) return 2;  // General initialization failed
    return 0;  // Everything okay
}