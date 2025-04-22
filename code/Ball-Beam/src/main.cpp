#include <Arduino.h>    // Arduino Core
#include <Eventually.h> // Event Driven Framework
#include "BallBeam/Controller.hpp"
#include "BallBeam/DistanceSensor.hpp"
#include "BallBeam/Stepper.hpp"
#include "BallBeam/Test.hpp"

// Pin Definitions
const uint8_t LEFT_BUTTON_PIN = 2;  
const uint8_t CENTER_BUTTON_PIN = 3;
const uint8_t RIGHT_BUTTON_PIN = 4;
const uint8_t LEFT_LIMIT_PIN = 5;
const uint8_t RIGHT_LIMIT_PIN = 6;
const uint8_t STEPPER_STEP_PIN = 8;
const uint8_t STEPPER_DIR_PIN = 9;
const uint8_t DISTANCE_SENSOR_XSHUT_PIN = A1;
const uint8_t POTENTIOMETER_PIN = A7;           // for testing

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
    50.0f,  // LEFT position in mm
    125.0f,    // CENTER position in mm
    200.0f    // RIGHT position in mm
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
DistanceSensor distanceSensor(DISTANCE_SENSOR_XSHUT_PIN, LOOP_PERIOD_MS); // uses I2C
Controller controller(LOOP_PERIOD_MS);
EvtManager mgr;

// Global State
// TargetPosition currentTarget = CENTER;
TargetPosition currentTarget = LEFT; // for testing

float prev_pos = 0.0f;
float buffer[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // for testing

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
    pinMode(POTENTIOMETER_PIN, INPUT); 
}

bool handleTimer(EvtListener* listener, EvtContext* ctx) {

    float position = distanceSensor.get_distance();
    // Serial.println(position); // for testing
    float filtered_pos = position; // for testing
    // float filtered_pos = (position + prev_pos) / 2.0f; 
    // prev_pos = position;
    // Serial.println(filtered_pos);

    float targetPosition = TARGET_POSITIONS[currentTarget];
    float error = targetPosition - filtered_pos;
    Serial.print("Error: ");
    Serial.println(error);

    float kp = 0.05; // Proportional gain
    float target_angle_deg = kp * error; // for testing
    float target_angle_steps = target_angle_deg * STEPS_PER_REV / 360.0f;
    float curr_angle_steps = stepper.get_step_count() % STEPS_PER_REV;
    float target_steps_per_sec = (target_angle_steps - curr_angle_steps) / (LOOP_PERIOD_MS / 1000.0f);
    // float target_steps_per_sec = kp * error; // for testing
    // Serial.print("Target Steps/sec: ");
    // Serial.println(target_steps_per_sec);
    // float target_angle_deg = controller.compute(filtered_pos, targetPosition);
    // float target_angle_steps = target_angle_deg * STEPS_PER_REV / 360.0f;
    // float curr_angle_steps = stepper.get_step_count() % STEPS_PER_REV;
    // float target_steps_per_sec = (target_angle_steps - curr_angle_steps) / (LOOP_PERIOD_MS / 1000.0f);
    
    stepper.set_steps_per_sec(target_steps_per_sec);
    stepper.start_stepping();
    
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

// *************************************
// * For Testing Purposes Only *
// *************************************
// void setup() {
//     Serial.begin(115200); 
//     while(!Serial) {
//         ; // Wait for serial port to connect. Needed for native USB port only
//     }

//     Serial.println("hello world");
// }

// void loop() {
//     // Test::test_distance_sensor(distanceSensor, LOOP_PERIOD_MS);
//     Test::test_stepper(stepper, STEPS_PER_REV);
//     delay(10);
// }
// *************************************

void setup() {
    Serial.begin(115200); 
    while(!Serial) {
        ; // Wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("hello world");
    pin_setup();
    distanceSensor.initialize(); // Initialize the distance sensor (maybe put in check on distance sensor first measurement)
    mgr.addListener(&timer);
    mgr.addListener(&leftBtn);
    mgr.addListener(&centerBtn);
    mgr.addListener(&rightBtn);
    mgr.addListener(&leftLimit);
    mgr.addListener(&rightLimit);
}

void loop() {
    mgr.loopIteration();
}