#include <Stepper.hpp>

Stepper::Stepper(uint8_t step_pin, uint8_t dir_pin, uint16_t steps_per_rev)
    : step_pin_{step_pin}, dir_pin_{dir_pin}, steps_per_rev_{steps_per_rev},
      steps_per_sec_{0}, direction_{Direction::CW}, step_count_{0} {}

void Stepper::set_steps_per_sec(uint32_t steps_per_sec) { steps_per_sec_ = steps_per_sec; }
void Stepper::set_direction(Direction direction) { direction_ = direction; }
void Stepper::reset_step_count() { step_count_ = 0; }
uint32_t Stepper::get_step_count() const { return step_count_; }

void Stepper::start_stepping() { /* Implement Later */}
void Stepper::stop_stepping() { /* Implement Later */}
