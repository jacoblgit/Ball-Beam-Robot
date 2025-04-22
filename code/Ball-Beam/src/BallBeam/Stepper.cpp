#include "Stepper.hpp"

// Global pointer to Stepper object for interrupt handler access
static Stepper* g_stepper_p = nullptr;

Stepper::Stepper(uint8_t step_pin, uint8_t dir_pin, uint16_t steps_per_rev)
    : step_pin_{step_pin}, dir_pin_{dir_pin}, steps_per_rev_{steps_per_rev},
      steps_per_sec_{0}, direction_{Direction::CW}, step_count_{0} {
        g_stepper_p = this;
      }

void Stepper::set_steps_per_sec(int32_t steps_per_sec) { 
  if (steps_per_sec < 0) {
    set_direction(Direction::CCW); 
    steps_per_sec_ = -steps_per_sec; // Make positive
  }
  else {
    set_direction(Direction::CW);
    steps_per_sec_ = steps_per_sec;
  }
}
void Stepper::set_direction(Direction direction) { direction_ = direction; }
void Stepper::reset_step_count() { step_count_ = 0; }
int32_t Stepper::get_step_count() const { return step_count_; }


void Stepper::start_stepping() {
  // Configure direction pin according to current direction setting
  digitalWrite(dir_pin_, direction_ == Direction::CW ? HIGH : LOW);
  
  // Validate stepping rate before proceeding
  // steps_per_sec_ must be greater than 3 to avoid overflow in the calculation
  if (steps_per_sec_ < 3) {
    stop_stepping();
    return;
  }
  if (steps_per_sec_ > MAX_STEPS_PER_SEC) {
    steps_per_sec_ = MAX_STEPS_PER_SEC; // Cap to maximum
  }
  
  // Enable GCLK for TC3
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |     // Enable clock
  GCLK_CLKCTRL_GEN_GCLK0 | // Use GCLK0 (48MHz)
  GCLK_CLKCTRL_ID_TCC2_TC3; // Select TC3
  while (GCLK->STATUS.bit.SYNCBUSY);  // Wait for sync

  // Disable timer for configuration
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Reset TC3
  TC3->COUNT16.CTRLA.bit.SWRST = 1;  // Software reset
  while (TC3->COUNT16.CTRLA.bit.SWRST); // Wait for reset to complete
  
  // Configure timer: 16-bit mode, match frequency waveform generation, 8x prescaler
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV256;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Calculate and set period based on requested step frequency
  // SystemCoreClock/256 gives timer tick frequency in Hz
  TC3->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / 256 / steps_per_sec_ - 1);
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Configure interrupt with medium priority
  NVIC_SetPriority(TC3_IRQn, 1); // Set priority to medium (1)
  // NVIC_SetPriority(TC3_IRQn, 0);
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