#include <Arduino.h>    // Arduino Core
#include <Eventually.h> // Event Driven Framework

EvtManager mgr;

void setup() {
}

void loop() {
    mgr.loopIteration();
    
}