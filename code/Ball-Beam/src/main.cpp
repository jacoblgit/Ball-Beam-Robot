#include <Arduino.h>    // Arduino Core
#include <Eventually.h> // Event Driven Framework
#include "BallBeam/Controller.hpp"
#include "BallBeam/DistanceSensor.hpp"
#include "BallBeam/Stepper.hpp"
#include "BallBeam/Test.hpp"

// Pin Definitions
// const uint8_t LEFT_BUTTON_PIN = 2;  
// const uint8_t CENTER_BUTTON_PIN = 3;
// const uint8_t RIGHT_BUTTON_PIN = 4;
// const uint8_t RIGHT_LIMIT_PIN = 5;
const uint8_t LEFT_LIMIT_PIN = 6;
const uint8_t STEPPER_STEP_PIN = 8;
const uint8_t STEPPER_DIR_PIN = 9;
const uint8_t DISTANCE_SENSOR_XSHUT_PIN = A1;
// const uint8_t POTENTIOMETER_PIN = A7;           // for testing

// Constants
const uint32_t LOOP_PERIOD_MS = 25;
const uint16_t STEPS_PER_REV = 1600; 
const uint32_t DEBOUNCE_MS = 70;
const float    STEPPER_RIGHT_LIMIT_RAD = 0.22;   //  16.8 degrees
const float    STEPPER_LEFT_LIMIT_RAD = -1 * STEPPER_RIGHT_LIMIT_RAD;   // -16.8 degrees

const float g = 9.8;                       // acceleration due to gravity (m/s^2)
// const float c_inertia = 2.0f / 3.0f;        // inertia constant for ping pong ball
const float c_inertia = 2.0f / 5.0f;            // inertia constant for steel ball
// const float tc = 8;                             // BEST (old wrong way)
const float tc = 1;                         // BEST (new way)      
// const float tc = 0.5;                         // test (new way)      


// Target position enum and mapping to actual positions
enum TargetPosition {
    LEFT,
    CENTER,
    RIGHT
};

const float TARGET_POSITIONS[] = {
    // all in meters
    0.075f,     // LEFT position
    0.125f,     // CENTER position
    0.175f      // RIGHT position
};

// Function Declarations
void pin_setup();
bool handleTimer(EvtListener* listener, EvtContext* ctx);
// bool handleLeftButton(EvtListener* listener, EvtContext* ctx);
// bool handleCenterButton(EvtListener* listener, EvtContext* ctx);
// bool handleRightButton(EvtListener* listener, EvtContext* ctx);
bool handleLeftLimit(EvtListener* listener, EvtContext* ctx);
// bool handleRightLimit(EvtListener* listener, EvtContext* ctx);

// Global Components
Stepper stepper(STEPPER_STEP_PIN, STEPPER_DIR_PIN,
    STEPS_PER_REV, STEPPER_LEFT_LIMIT_RAD, STEPPER_RIGHT_LIMIT_RAD);
DistanceSensor distanceSensor(DISTANCE_SENSOR_XSHUT_PIN, LOOP_PERIOD_MS); // uses I2C
Controller controller(LOOP_PERIOD_MS);
EvtManager mgr;

// Global State
TargetPosition currentTarget = CENTER;

// float alpha = 0.09f; // EMA filter coefficient (between 0 and 1, lower is more smoothing, higher is more responsive)
float alpha = 0.08f; // EMA filter coefficient (between 0 and 1, lower is more smoothing)
float prev_filtered_pos; //= TARGET_POSITIONS[currentTarget]; // Initialize with the target position
float prev_error; //= 0.0f; // Previous error for derivative calculation
// float cum_error = 0.0f; // Cumulative error for integral calculation

// Global Event Listeners
EvtTimeListener timer(LOOP_PERIOD_MS, true, handleTimer);
// EvtPinListener leftBtn(LEFT_BUTTON_PIN, DEBOUNCE_MS, LOW, handleLeftButton);
// EvtPinListener centerBtn(CENTER_BUTTON_PIN, DEBOUNCE_MS, LOW, handleCenterButton);
// EvtPinListener rightBtn(RIGHT_BUTTON_PIN, DEBOUNCE_MS, LOW, handleRightButton);
EvtPinListener leftLimit(LEFT_LIMIT_PIN, DEBOUNCE_MS, HIGH, handleLeftLimit);
// EvtPinListener rightLimit(RIGHT_LIMIT_PIN, DEBOUNCE_MS, HIGH, handleRightLimit);

// Function Implementations
void pin_setup() {
    // pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
    // pinMode(CENTER_BUTTON_PIN, INPUT_PULLUP);
    // pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
    // pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    // pinMode(POTENTIOMETER_PIN, INPUT); 
}

bool handleTimer(EvtListener* listener, EvtContext* ctx) {

    // get distance from target (error)
    float position = distanceSensor.get_distance() * 0.001;                     // in meters
    float filtered_pos = alpha * position + (1 - alpha) * prev_filtered_pos;    // smoothing
    prev_filtered_pos = filtered_pos;
    // Serial.println(filtered_pos*100, 1);

    float targetPosition = TARGET_POSITIONS[currentTarget];
    float error = targetPosition - filtered_pos;
    // Serial.print("error: ");
    Serial.print(error*1000);
    Serial.print(", ");

    // get derivative of error
    float derror = (error - prev_error) / (LOOP_PERIOD_MS * 0.001f);            // derivative of error (meters/sec)
    prev_error = error;
    // Serial.print("derror: ");
    // Serial.println(derror, 4);

    float kd_adjustment = 1.0f;                         // damping factor is proportional to kd. use adjustment to make damping factor 1 (BEST)
    float kp_adjustment = 1.0f;                         // tc is inversely proportional to kp

    // Serial.print("kp: ");
    // Serial.println(kp, 4);  
    // Serial.print("kd: ");
    // Serial.println(kd, 4);
    float kd = (1 * 2 * (1 + c_inertia) / (g * tc)) * kd_adjustment;     // derivative gain
    float kp = (1 * (1 + c_inertia) / (g * tc * tc)) * kp_adjustment;    // proportional gain
    float target_angle_rad = kp * error + kd * derror; // in radians
    // Serial.print("target angle: ");
    Serial.println(target_angle_rad * 180 / PI, 2);

    // tuning for ping pong ball
    // float kp = -0.00040;  
    // float kd = -0.00025;
    
    // tuning for steal ball
    // float kp = -0.00030;
    // float kd = 0;

    float target_angle_steps = target_angle_rad * STEPS_PER_REV / (2.0 * PI);
    float curr_angle_steps = stepper.get_step_count() % STEPS_PER_REV;
    float target_steps_per_sec = (target_angle_steps - curr_angle_steps) / (LOOP_PERIOD_MS / 1000.0f);
    // Serial.print("target steps/sec: ");
    // Serial.println(target_steps_per_sec, 2);
    // Serial.println();

    // // stop if at target
    // if (abs(error) <= 10.0f && abs(derror) <= 25.0f) {
    //     stepper.stop_stepping();
    //     return true;
    // }
    
    stepper.set_steps_per_sec(target_steps_per_sec);
    stepper.start_stepping();
    return true;
}

// bool handleLeftButton(EvtListener* listener, EvtContext* ctx) {
//     currentTarget = LEFT;
//     return true;
// }

// bool handleCenterButton(EvtListener* listener, EvtContext* ctx) {
//     currentTarget = CENTER;
//     return true;
// }

// bool handleRightButton(EvtListener* listener, EvtContext* ctx) {
//     currentTarget = RIGHT;
//     return true;
// }

bool handleLeftLimit(EvtListener* listener, EvtContext* ctx) {
    stepper.stop_stepping();
    Serial.println("Left limit reached");
    while (true) { delay(1000);} // stop program
    return true;
}

// bool handleRightLimit(EvtListener* listener, EvtContext* ctx) {
//     stepper.stop_stepping();
//     Serial.println("Right limit reached");
//     while (true) { delay(1000);} // stop program
//     return true;
// }

void calibrate_stepper() {
    // runs track up against the right limit switch
    // requires correct STEPPER_RIGHT_LIMIT_RAD value
    stepper.set_direction(Direction::CCW);
    while(!digitalRead(LEFT_LIMIT_PIN)) {
        stepper.single_step();
        delay(25);
    }
    stepper.set_step_count(STEPPER_LEFT_LIMIT_RAD * STEPS_PER_REV / (2 * PI));
    stepper.set_steps_per_sec(-STEPPER_LEFT_LIMIT_RAD * STEPS_PER_REV / (2 * PI) / 2);
    stepper.start_stepping();
    delay(2000);
    stepper.stop_stepping();
    Serial.println("Calibration complete");
}

void setup() {
    Serial.begin(115200); 
    while(!Serial) {
        ; // Wait for serial port to connect.
    }
    Serial.println("hello world");
    pin_setup();
    distanceSensor.initialize(); // Initialize the distance sensor (maybe put in check on distance sensor first measurement)
    mgr.addListener(&timer);
    // mgr.addListener(&leftBtn);
    // mgr.addListener(&centerBtn);
    // mgr.addListener(&rightBtn);
    mgr.addListener(&leftLimit);
    // mgr.addListener(&rightLimit);

    calibrate_stepper();

    Serial.println("Setup complete.");
    delay(1000);

    // initialize values
    prev_filtered_pos = distanceSensor.get_distance() * 0.001; // in meters
    
    float targetPosition = TARGET_POSITIONS[currentTarget];
    prev_error = targetPosition - prev_filtered_pos;
    
    // Testing
    // Test::test_stepper(stepper, STEPS_PER_REV);
}

void loop() {
    mgr.loopIteration();
}