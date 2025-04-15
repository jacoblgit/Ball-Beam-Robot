#ifndef TEST_HPP
#define TEST_HPP

#include <Arduino.h>
#include "Stepper.hpp"
#include "DistanceSensor.hpp"

namespace Test {
    // Test function for stepper motor
    void test_stepper(Stepper& stepper, uint16_t steps_per_rev);    
    void test_distance_sensor(DistanceSensor& sensor, uint32_t LOOP_PERIOD_MS);
}

#endif // TEST_HPP
