#ifndef io_cpp
#define io_cpp

#include <Arduino.h>
#include <Wire.h>
#include "io.h"
#include "DEBUG.h"
#include "adc.h"

//vars for chanel current limit in mA
uint16_t ch1_current_limit = 0;
uint16_t ch2_current_limit = 0;
uint16_t ch3_current_limit = 0;
uint16_t ch4_current_limit = 0;
uint16_t ch5_current_limit = 0;
uint16_t ch6_current_limit = 0;
uint16_t ch7_current_limit = 0;
uint16_t ch8_current_limit = 0;
uint16_t ch9_current_limit = 0;
uint16_t ch10_current_limit = 0;
uint16_t ch11_current_limit = 0;
uint16_t ch12_current_limit = 0;
uint16_t ch13_current_limit = 0;
uint16_t ch14_current_limit = 0;
uint16_t ch15_current_limit = 0;
uint16_t ch16_current_limit = 0;
uint16_t peripheral_current_limit = 0;

//iox interrupt register values
uint8_t iox_0_port_0_interrupt = 0xFF;
uint8_t iox_0_port_1_interrupt = 0xFF;

//specal io modes
bool io_allow_off_road = false;
bool io_blackout_enabled = false;

//cue for returning io channel state
// Create a queue to hold one uint8_t value
QueueHandle_t channelStateQueue = xQueueCreate(1, sizeof(uint32_t));

/**
 * Reads the current input/output state from the specified port and IOX number.
 *
 * @param port The port number (0 or 1) to read from.
 * @param iox_num The IOX number (0 or 1) to read from.
 * @return The current IO state as a uint8_t value.
 */
uint8_t io_read_current_io_state(uint8_t port) {

  debug_msg(DEBUG_PARTIAL_IO, "io_read_current_io_state called, reading current IO state of given io expander", false, 0);

  uint8_t iox_input_port_register = (port == 0) ? iox_input_port_0 : iox_input_port_1;

  Wire.beginTransmission(iox_0_add);
  Wire.write(iox_input_port_register);
  Wire.endTransmission();

  Wire.requestFrom(iox_0_add, static_cast<uint8_t>(1));
  return Wire.read();
}

/**
 * @brief Performs read or write operations on a pin.
 * 
 * This function allows you to read or write to a pin, either onboard or offboard.
 * 
 * @param pin_needed The pin structure containing information about the pin to be operated on.
 * @param read_write Specifies whether to perform a read or write operation.
 * @param high_low Specifies whether to set the pin high or low (applicable for write operations).
 * @return If performing a read operation, returns the value read from the pin. If performing a write operation, returns 0.
 *         If an invalid `read_write` or `high_low` value is provided, the function does nothing and returns 0.
 */
int IRAM_ATTR io_call(struct pin pin_needed, enum read_write read_write, enum high_low high_low) {

  debug_msg(DEBUG_PARTIAL_IO, "io_call called, making pin call", false, 0);

  if (pin_needed.onboard) {
    if (read_write == WRITE) {
      if (high_low == high) {
        digitalWrite(pin_needed.pin_number, HIGH);

      } else if (high_low == low) {
        digitalWrite(pin_needed.pin_number, LOW);
        
      }
    } else if (read_write == READ) {
      int read_value = digitalRead(pin_needed.pin_number);
      return read_value;

    }

  } else {
    if (read_write == WRITE) {
      uint8_t current = io_read_current_io_state(pin_needed.port);
      uint8_t output = 0b00000000;
      uint8_t mask_not = ~pin_needed.mask;

      if (high_low == high) {
        output = current | pin_needed.mask;

      } else if (high_low == low) {
        output = current & mask_not;

      }

      Wire.beginTransmission(iox_0_add);


      if (pin_needed.port == 0) {
        Wire.write(iox_output_port_0);

      } else {
        Wire.write(iox_output_port_1);

      }

      Wire.write(output);
      Wire.endTransmission();

    } else if (read_write == READ) {

      uint8_t readval = 0;
      uint8_t output_byte = 0;

      Wire.beginTransmission(iox_0_add);

      if (pin_needed.port == 0) {
        Wire.write(iox_input_port_0);

      } else if (pin_needed.port == 1) {
        Wire.write(iox_input_port_1);

      }

      Wire.endTransmission();
      
      Wire.requestFrom(iox_0_add, static_cast<uint8_t>(1));

      readval = Wire.read();
      output_byte = readval & pin_needed.mask;

      if (output_byte == 0) {
        return LOW;

      } else {
        return HIGH;

      }
    }
  }
  return 0;
}



/**
 * @brief Wrapper function to call io_call with parameters passed via a struct pointer.
 *
 * This function is intended to be used as a wrapper for RTOS tasks or threads that require
 * passing multiple parameters. It casts the provided void pointer to the expected parameter
 * structure, calls the io_call function with the extracted parameters, and then frees the
 * allocated memory for the parameter structure.
 *
 * @param peram Pointer to a dynamically allocated io_call_rtos_wraper_params structure containing
 *              the parameters required by io_call.
 * @return int  The result returned by the io_call function.
 *
 * @note The function takes ownership of the parameter pointer and deletes it after use.
 */
void io_call_rtos_wraper(void *peram) {


  // Cast the parameter to the correct type
  io_call_rtos_wraper_params *params = static_cast<io_call_rtos_wraper_params *>(peram);

  // Call the io_call function with the provided parameters
  io_call(params->pin_needed, params->rw, params->hl);

  // Free the allocated memory for the params structure
  delete params;

  vTaskDelete(NULL); // Delete the task after completion
}

/**
 * @brief Initializes the GPIO pins and I2C interface for the system.
 *
 * This function iterates through the configured pin definitions and sets up each pin
 * according to its mode (input, output, I2C, interrupt, analog input, or empty).
 * - Onboard pins are initialized using pinMode and, if required, attachInterrupt.
 * - Offboard output pins are configured via IO expander (IOX) registers.
 * - I2C pins are detected and initialized with the Wire library at 400kHz.
 * - All IOX outputs are set low by default.
 * - IOX pin modes are written to the appropriate configuration registers.
 *
 * Debug messages are generated throughout the process for tracing and error handling.
 *
 * @note This function should be called during system initialization before using any GPIO or I2C functionality.
 */
void io_gpio_init() {

  debug_msg(DEBUG_PARTIAL_IO, "io_gpio_init called, setting up pins", false, 0);

  //init vars
  uint8_t iox_0_port_0_pinmode = 0x00;
  uint8_t iox_0_port_1_pinmode = 0x00;

  //i2c setup holding
  int i2c_sda = -1;
  int i2c_scl = -1;

  //begin iterating through pins
  for (size_t i = 1; i < sizeof(pin_names[0]); i++) {

    switch (pin_names[i]->pin_mode) {
    default:
      // Error handling for unexpected pin_mode value
      debug_msg(DEBUG_PARTIAL_IO, "Unexpected pin_mode value", false, 0);
      break;
    case in:
      if (pin_names[i]->onboard) {
        pinMode(pin_names[i]->pin_number, INPUT);

      }
      //no need to do anything to the iox, all pins default to input
      break;
    
    case out:
      if (pin_names[i]->onboard) {
        pinMode(pin_names[i]->pin_number, OUTPUT);

      } else {
        if (pin_names[i]->port == 0) {
          iox_0_port_0_pinmode |= pin_names[i]->mask;

        } else {
          iox_0_port_1_pinmode |= pin_names[i]->mask;

        }
      }
      
      break;

    case i2c: //setup i2c pins
      if (pin_names[i]->pin_mode == i2c) {
        if (i2c_sda == -1) {
          if (pin_names[i]->port == 2) {
            i2c_sda = pin_names[i]->pin_number;
          }
        } else if (i2c_scl == -1) {
          if (pin_names[i]->port == 3) {
            i2c_scl = pin_names[i]->pin_number;
          }
        }
        
        if (i2c_sda != -1 && i2c_scl != -1) {
          // If both SDA and SCL are set, initialize I2C
          Wire.begin(i2c_sda, i2c_scl);
          Wire.setClock(400000);
          debug_msg(DEBUG_PARTIAL_IO, "I2C initialized & started", false, 0);
        }
      }
      break;
    case intr: //setup interrupts for checking when io state is changed
      if (pin_names[i]->onboard) {
        pinMode(pin_names[i]->pin_number, INPUT_PULLUP);
        // Use a static function for the interrupt handler
        attachInterrupt(
          digitalPinToInterrupt(pin_names[i]->pin_number),
          []() { /* Call a static/global handler here, or set a flag for processing in loop() */ },
          FALLING
        );
        // NOTE: Replace the lambda body with a call to a static/global handler as needed.
        // If you need to pass pin info, store it in a global variable or use a lookup.
      }
      //no need to do anything to the iox, all pins are being used as outputs
      break;
    case empty_pin:
      //do nothing
      break;
    case analog_in:
      if (pin_names[i]->onboard) {
        pinMode(pin_names[i]->pin_number, INPUT);
        
      } 
      //no need to do anything ans offboard analouge pins are initialized elsewhere
      break;
    }  
  }

  //set all outputs of iox low
  //write to output at port 0
  Wire.beginTransmission(iox_0_add);
  Wire.write(iox_output_port_0);
  Wire.write(0x00);
  Wire.endTransmission();

  //write to output at port 1
  Wire.beginTransmission(iox_0_add);
  Wire.write(iox_output_port_1);
  Wire.write(0x00);
  Wire.endTransmission();


  //write pinmode to port 0 
  Wire.beginTransmission(iox_0_add);
  Wire.write(iox_config_port_0);
  Wire.write(iox_0_port_0_pinmode);
  Wire.endTransmission();

  //write pinmode to port 1 
  Wire.beginTransmission(iox_0_add);
  Wire.write(iox_config_port_1);
  Wire.write(iox_0_port_1_pinmode);
  Wire.endTransmission();

  debug_msg(DEBUG_PARTIAL_IO, "pins have been initialized", false, 0);

  return;
}

/**
 * @brief Reads the current state of all IO channels and sends the result to a FreeRTOS queue.
 *
 * This function communicates with an IO expander over I2C to read the state of two input ports,
 * reverses the bit order of each byte, and combines them into a single 32-bit integer. It also
 * appends the state of a peripheral power gate channel to the result. The final state is sent
 * to the provided FreeRTOS queue. The task deletes itself upon completion.
 *
 * @param peram Pointer to a FreeRTOS queue handle (QueueHandle_t) where the channel state will be sent.
 *
 * @note
 * - The function is intended to be run as a FreeRTOS task.
 */
void io_read_chanel_state (void *peram) {

  debug_msg(DEBUG_PARTIAL_IO, "io_read_chanel_state called, reading channel state", false, 0);

  QueueHandle_t queue = (QueueHandle_t)peram;

    // create var to hold the state
    uint32_t state = 0;

    // Read the current state of all channels
    Wire.beginTransmission(iox_0_add);
    Wire.write(iox_input_port_0);
    Wire.endTransmission();
    Wire.requestFrom(iox_0_add, static_cast<uint8_t>(2));
    if (Wire.available() == 2) {
      // Read the two bytes of input state
      uint8_t port0_state = Wire.read();
      uint8_t port1_state = Wire.read();
      Wire.endTransmission();

      // Reverse the bit order of each byte
      auto reverse_byte = [](uint8_t b) -> uint8_t {
        b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
        b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
        b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
        return b;
      };
      port0_state = reverse_byte(port0_state);
      port1_state = reverse_byte(port1_state);

      // Combine the two bytes into a single 32-bit integer
      state = static_cast<uint32_t>(port0_state) | (static_cast<uint32_t>(port1_state) << 8);
      //apend periferal chanel data
      state |= (static_cast<uint32_t>(static_cast<bool>(io_call(peripheral_pwr_gate, READ, read_mode))) << 16);
    } else {
      debug_msg(DEBUG_PARTIAL_IO, "Failed to read channel state from IOX", false, 0);
      state = 0; // Set to zero if reading fails
    }

    // Send the result to the queue
    xQueueSend(queue, &state, portMAX_DELAY);

    vTaskDelete(NULL); // Delete the task after completion
}



#endif // io_cpp