#ifndef TEST_HPP
#define TEST_HPP

#include <Arduino.h>
#include "Stepper.hpp"

namespace Test {
    // Test function for stepper motor
    void test_stepper(Stepper& stepper, uint16_t steps_per_rev);    
}

#endif // TEST_HPP
