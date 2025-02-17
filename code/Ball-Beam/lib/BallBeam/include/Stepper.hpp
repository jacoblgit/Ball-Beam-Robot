#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <Arduino.h>

enum class Direction { CW, CCW };

class Stepper {
    public:
        Stepper(uint8_t step_pin, uint8_t dir_pin, uint16_t steps_per_rev);
        void set_steps_per_sec(uint32_t steps_per_sec);
        void set_direction(Direction direction);
        void reset_step_count();
        uint32_t get_step_count() const;
        void start_stepping();
        void stop_stepping();

    private:
        const uint8_t step_pin_;
        const uint8_t dir_pin_;
        const uint16_t steps_per_rev_;
        uint32_t steps_per_sec_;
        Direction direction_;
        uint32_t step_count_;
};

#endif