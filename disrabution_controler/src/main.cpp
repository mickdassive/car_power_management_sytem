#include <Arduino.h>
#include <Wire.h>
#include "adc.h"
#include "DEBUG.h"
#include "io.h"
#include "can.h"


void setup() {
  // start the serial communication
  Serial.begin(115200);

  // init gpio and adc
  io_gpio_init();
  adc_init(true);

  // init can bus
  can_init();

  //rtos task setup
  xTaskCreate(
    adc_monitor_periral_ch_current, // Task function
    "peripheral power bus monitoring", // Name of the task
    2048, // Stack size in words
    NULL, // Task parameters
    30, // Task priority
    NULL // Task handle
  );

  xTaskCreate(
    can_check_for_alerts, // Task function
    "can alert handler", // Name of the task
    2048, // Stack size in words
    NULL, // Task parameters
    1, // Task priority
    NULL // Task handle
  );
}

void loop() {
  //



}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}