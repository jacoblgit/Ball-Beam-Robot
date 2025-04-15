#include "test.hpp"

namespace Test {
    void test_stepper(Stepper& stepper, uint16_t steps_per_rev) {
        Serial.println("Starting stepper motor test...");
        
        // Reset step count
        stepper.reset_step_count();
        
        // Print initial step count
        Serial.print("Initial step count: ");
        Serial.println(stepper.get_step_count());
        
        // Set direction to clockwise and start stepping
        Serial.println("Setting direction to CW and starting motor...");
        stepper.set_direction(Direction::CW);
        stepper.set_steps_per_sec(steps_per_rev);
        stepper.start_stepping();
        
        // Wait 1 second
        delay(1000);
        
        // Print step count after 1 second of CW rotation
        Serial.print("Step count after 1 second CW: ");
        Serial.println(stepper.get_step_count());
        
        // Set direction to counter-clockwise and continue stepping
        Serial.println("Setting direction to CCW and continuing...");
        stepper.set_direction(Direction::CCW);
        stepper.start_stepping();
        
        // Wait 1 more second
        delay(1000);
        
        // Stop stepping
        stepper.stop_stepping();
        
        // Print final step count
        Serial.print("Final step count after 1 second CCW: ");
        Serial.println(stepper.get_step_count());
        
        Serial.println("Stepper motor test complete.");
    }

    void test_distance_sensor(DistanceSensor& sensor, uint32_t LOOP_PERIOD_MS) {        
        // Initialize the sensor
        sensor.initialize();
        
        // print sensor value
        float distance = sensor.get_distance();
        if (distance >= 0) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
        } else {
            Serial.println("Invalid distance reading");
        }
        delay(LOOP_PERIOD_MS);
    }
}