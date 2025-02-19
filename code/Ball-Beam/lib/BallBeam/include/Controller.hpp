#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Arduino.h>

class Controller {
public:
    explicit Controller(uint32_t loop_period_ms);
    
    // Compute control output based on current position and target
    float compute(float current_position, float target_position);
    
    // Setters for gains
    void set_p_gain(float kp);
    void set_i_gain(float ki);
    void set_d_gain(float kd);

private:
    const uint32_t loop_period_ms_;
    
    // Control gains
    float kp_ = 1.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    
    // State for I and D terms
    float integral_ = 0.0f;
    float prev_error_ = 0.0f;
    bool first_sample_ = true;
};

#endif // CONTROLLER_HPP