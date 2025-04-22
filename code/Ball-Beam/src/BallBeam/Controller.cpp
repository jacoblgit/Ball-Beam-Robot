#include "Controller.hpp"

Controller::Controller(uint32_t loop_period_ms)
    : loop_period_s_(loop_period_ms / 1000.0f),
      kp_(0.5f),
      ki_(0.0f),
      kd_(0.0f),
      integral_(0.0f),
      prev_error_(0.0f),
      first_sample_(true) {
}

float Controller::compute(float current_position, float target_position) {
    // Calculate error (setpoint - process variable)
    float error = target_position - current_position;
    
    // Calculate proportional term
    float p_term = kp_ * error;
    
    // Calculate integral term
    integral_ += error * loop_period_s_;
    float i_term = ki_ * integral_;
    
    // Calculate derivative term
    float d_term = 0.0f;
    if (!first_sample_) {
        // Calculate change in error, convert to rate per second
        float error_rate = (error - prev_error_) / loop_period_s_;
        d_term = kd_ * error_rate;
    } else {
        first_sample_ = false;
    }
    
    // Store current error for next derivative calculation
    prev_error_ = error;
    
    Serial.print("P Term: ");
    Serial.print(p_term);
    Serial.print(" I Term: ");
    Serial.print(i_term);
    Serial.print(" D Term: ");
    Serial.print(d_term);
    Serial.print(" total: ");
    Serial.println(p_term + i_term + d_term);

    // Combine P, I, and D terms for final control output
    float control = p_term + i_term + d_term;
    return control;
}

void Controller::set_p_gain(float kp) { kp_ = kp; }
void Controller::set_i_gain(float ki) { ki_ = ki; }
void Controller::set_d_gain(float kd) { kd_ = kd; }