#include <Arduino.h>    // Arduino Core
#include <Eventually.h> // Event Driven Framework
#include <Wire.h>       // I2C communication
#include "BallBeam/Controller.hpp"
#include "BallBeam/DistanceSensor.hpp"
#include "BallBeam/Stepper.hpp"

// Pin Definitions
const uint8_t LEFT_BUTTON_PIN = 2;  
const uint8_t CENTER_BUTTON_PIN = 3;
const uint8_t RIGHT_BUTTON_PIN = 4;
const uint8_t LEFT_LIMIT_PIN = 5;
const uint8_t RIGHT_LIMIT_PIN = 6;
const uint8_t STEPPER_STEP_PIN = 8;
const uint8_t STEPPER_DIR_PIN = 9;
const uint8_t DISTANCE_SENSOR_XSHUT_PIN = A1;

// Constants
const uint32_t LOOP_PERIOD_MS = 100;
const uint16_t STEPS_PER_REV = 1600; 
const uint32_t DEBOUNCE_MS = 40;

// Target position enum and mapping to actual positions
enum TargetPosition {
    LEFT,
    CENTER,
    RIGHT
};

const float TARGET_POSITIONS[] = {
    -5.0f,  // LEFT position in cm
    0.0f,    // CENTER position in cm
    5.0f    // RIGHT position in cm
};

// Function Declarations
void pin_setup();
bool handleTimer(EvtListener* listener, EvtContext* ctx);
bool handleLeftButton(EvtListener* listener, EvtContext* ctx);
bool handleCenterButton(EvtListener* listener, EvtContext* ctx);
bool handleRightButton(EvtListener* listener, EvtContext* ctx);
bool handleLeftLimit(EvtListener* listener, EvtContext* ctx);
bool handleRightLimit(EvtListener* listener, EvtContext* ctx);

// Global Components
Stepper stepper(STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPS_PER_REV);
DistanceSensor distanceSensor(&Wire, DISTANCE_SENSOR_XSHUT_PIN, LOOP_PERIOD_MS);
Controller controller(LOOP_PERIOD_MS);
EvtManager mgr;

// Global State
TargetPosition currentTarget = CENTER;

// Global Event Listeners
EvtTimeListener timer(LOOP_PERIOD_MS, true, handleTimer);
EvtPinListener leftBtn(LEFT_BUTTON_PIN, DEBOUNCE_MS, LOW, handleLeftButton);
EvtPinListener centerBtn(CENTER_BUTTON_PIN, DEBOUNCE_MS, LOW, handleCenterButton);
EvtPinListener rightBtn(RIGHT_BUTTON_PIN, DEBOUNCE_MS, LOW, handleRightButton);
EvtPinListener leftLimit(LEFT_LIMIT_PIN, DEBOUNCE_MS, LOW, handleLeftLimit);
EvtPinListener rightLimit(RIGHT_LIMIT_PIN, DEBOUNCE_MS, LOW, handleRightLimit);

// Function Implementations
void pin_setup() {
    pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CENTER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
}

bool handleTimer(EvtListener* listener, EvtContext* ctx) {
    float position = distanceSensor.get_distance();
    float targetPosition = TARGET_POSITIONS[currentTarget];
    float control = controller.compute(position, targetPosition);
    stepper.set_steps_per_sec(control);  // Assuming control output maps to steps/sec
    return true;
}

bool handleLeftButton(EvtListener* listener, EvtContext* ctx) {
    currentTarget = LEFT;
    return true;
}

bool handleCenterButton(EvtListener* listener, EvtContext* ctx) {
    currentTarget = CENTER;
    return true;
}

bool handleRightButton(EvtListener* listener, EvtContext* ctx) {
    currentTarget = RIGHT;
    return true;
}

bool handleLeftLimit(EvtListener* listener, EvtContext* ctx) {
    stepper.stop_stepping();
    return true;
}

bool handleRightLimit(EvtListener* listener, EvtContext* ctx) {
    stepper.stop_stepping();
    return true;
}

// void setup() {
//     Serial.begin(9600);
//     pin_setup();

//     stepper.set_direction(Direction::CW); // Set initial direction
//     stepper.set_steps_per_sec(STEPS_PER_REV); // Set initial speed
//     stepper.start_stepping(); // Start stepping
// }

// void loop() {
//     delay(1000);
//     stepper.set_direction(Direction::CW); // Change direction to CW
//     stepper.start_stepping(); // Start stepping
//     delay(1000);
//     stepper.set_direction(Direction::CCW); // Change direction to CCW
//     stepper.start_stepping(); // Start stepping
// }

// void setup() {
//     pin_setup();
//     mgr.addListener(&timer);
//     mgr.addListener(&leftBtn);
//     mgr.addListener(&centerBtn);
//     mgr.addListener(&rightBtn);
//     mgr.addListener(&leftLimit);
//     mgr.addListener(&rightLimit);
// }

// void loop() {
//     mgr.loopIteration();
// }