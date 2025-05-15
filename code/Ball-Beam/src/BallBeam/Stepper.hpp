#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <Arduino.h>

// 100 steps/sec is 3.75 RPM at 1600 steps/rev
const uint32_t MAX_STEPS_PER_SEC = 100;
// const uint32_t MAX_STEPS_PER_SEC = 50;


enum class Direction { CW, CCW };

class Stepper {
    public:
        Stepper(uint8_t step_pin, uint8_t dir_pin,
            uint16_t steps_per_rev, float STEPPER_LEFT_LIMIT_RAD, float STEPPER_RIGHT_LIMIT_RAD);
        void set_steps_per_sec(int32_t steps_per_sec);
        void set_rad_per_sec(float rad_per_sec);
        void set_direction(Direction direction);
        void set_step_count(int32_t step_count);
        int32_t get_step_count() const;
        float get_angle() const;
        void start_stepping();
        void stop_stepping();
        void single_step();

    private:
        const uint8_t step_pin_;
        const uint8_t dir_pin_;
        const uint16_t steps_per_rev_;
        const float stepper_left_limit_rad_;
        const float stepper_right_limit_rad_;
        uint32_t steps_per_sec_;
        Direction direction_;
        int32_t step_count_;
        friend void TC3_Handler();
};

#endif