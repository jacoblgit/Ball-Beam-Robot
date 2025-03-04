#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Arduino.h>

class Controller {
public:
    explicit Controller(uint32_t loop_period_ms);
    
    // Compute control output based on current position and target
    float compute(float current_position, float target_position);
    
    // Setters for gains
    void set_p_gain(float kp) { kp_ = kp; }
    void set_i_gain(float ki) { ki_ = ki; }
    void set_d_gain(float kd) { kd_ = kd; }

private:
    const float loop_period_s_;  // Period in seconds
    
    // Control gains
    float kp_;
    float ki_;
    float kd_;
    
    // State for I and D terms
    float integral_;
    float prev_error_;
    bool first_sample_;
};

#endif // CONTROLLER_HPP