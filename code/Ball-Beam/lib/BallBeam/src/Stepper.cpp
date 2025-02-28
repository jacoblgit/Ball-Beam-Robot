#include <Stepper.hpp>

// Global pointer to Stepper object for interrupt handler access
static Stepper* g_stepper_p = nullptr;

Stepper::Stepper(uint8_t step_pin, uint8_t dir_pin, uint16_t steps_per_rev)
    : step_pin_{step_pin}, dir_pin_{dir_pin}, steps_per_rev_{steps_per_rev},
      steps_per_sec_{0}, direction_{Direction::CW}, step_count_{0} {
        g_stepper_p = this;
      }

void Stepper::set_steps_per_sec(uint32_t steps_per_sec) { steps_per_sec_ = steps_per_sec; }
void Stepper::set_direction(Direction direction) { direction_ = direction; }
void Stepper::reset_step_count() { step_count_ = 0; }
uint32_t Stepper::get_step_count() const { return step_count_; }


void Stepper::start_stepping() {
  // Configure direction pin according to current direction setting
  digitalWrite(dir_pin_, direction_ == Direction::CW ? HIGH : LOW);
  
  // Validate stepping rate before proceeding
  if (steps_per_sec_ == 0) {
      return;
  }
  
  // Configure TC3 timer for step pulse generation
  
  // Disable timer for configuration
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Configure timer: 16-bit mode, match frequency waveform generation, 8x prescaler
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV8;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Calculate and set period based on requested step frequency
  // SystemCoreClock/8 gives timer tick frequency in Hz
  TC3->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / 8 / steps_per_sec_ - 1);
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Configure interrupt with high priority
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);
  TC3->COUNT16.INTENSET.bit.MC0 = 1;
  
  // Enable timer to start operation
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void Stepper::stop_stepping() {
  // Disable TC3 timer
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Disable TC3 interrupt
  TC3->COUNT16.INTENCLR.bit.MC0 = 1;
  NVIC_DisableIRQ(TC3_IRQn);
  
  // Ensure step pin is LOW (inactive)
  digitalWrite(step_pin_, LOW);
}

void TC3_Handler() {
  // Verify and clear Match Counter 0 interrupt flag
  if (TC3->COUNT16.INTFLAG.bit.MC0) {
      TC3->COUNT16.INTFLAG.bit.MC0 = 1;
      
      // Ensure stepper instance exists
      if (g_stepper_p) {
          // Generate step pulse - HIGH
          digitalWrite(g_stepper_p->step_pin_, HIGH);
          
          // Maintain minimum pulse width (7 cycles ≈ 146ns @ 48MHz)
          asm volatile (
              "nop\n\t"
              "nop\n\t"
              "nop\n\t"
              "nop\n\t"
              "nop\n\t"
              "nop\n\t"
              "nop\n\t"
          );
          
          // Complete step pulse - LOW
          digitalWrite(g_stepper_p->step_pin_, LOW);
          
          // Update position counter according to direction
          if (g_stepper_p->direction_ == Direction::CW) {
              g_stepper_p->step_count_++;
          } else {
              g_stepper_p->step_count_--;
          }
      }
  }
}