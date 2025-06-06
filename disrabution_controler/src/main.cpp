#include <Arduino.h>
#include <Wire.h>
#include "adc.h"
#include "DEBUG.h"
#include "io.h"


void setup() {
  // start the serial communication
  Serial.begin(115200);

  // init gpio and adc
  io_gpio_init();
  adc_init(true);
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}