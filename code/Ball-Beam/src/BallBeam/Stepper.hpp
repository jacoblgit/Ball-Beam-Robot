#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <Arduino.h>

// 10000 steps/sec is 375 RPM at 1600 steps/rev
const uint32_t MAX_STEPS_PER_SEC = 10000;

enum class Direction { CW, CCW };

class Stepper {
    public:
        Stepper(uint8_t step_pin, uint8_t dir_pin, uint16_t steps_per_rev);
        void set_steps_per_sec(int32_t steps_per_sec);
        void set_direction(Direction direction);
        void reset_step_count();
        int32_t get_step_count() const;
        void start_stepping();
        void stop_stepping();

    private:
        const uint8_t step_pin_;
        const uint8_t dir_pin_;
        const uint16_t steps_per_rev_;
        uint32_t steps_per_sec_;
        Direction direction_;
        int32_t step_count_;
        friend void TC3_Handler();
};

#endif