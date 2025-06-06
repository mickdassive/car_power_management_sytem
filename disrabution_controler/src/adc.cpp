//{usbc car module}
//{adc.cpp}
//Copyright (C) {2023}  {mickmake}
//
//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.
//usbc car module
//15/09/2023
//Tokyo Andreana

#ifndef adc_cpp
#define adc_cpp

#include <Arduino.h>
#include <Wire.h>
#include "adc.h"
#include "DEBUG.h"
#include "io.h"


/**
 * @brief Performs self calibration for the ADC module.
 * 
 * This function starts a self calibration process for the ADC module. It reads the current configuration state
 * of the ADC, sets the self calibration bit, and writes the updated configuration back to the ADC. Then, it continuously
 * checks the self calibration bit until it is cleared, indicating that the calibration process is complete.
 * 
 */
void adc_self_cal (uint8_t adc_add) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_self_cal called beginnign selfcal seqwince", false, 0);

  // Local variables
  bool cal_complete = false;
  uint8_t self_cal_mask = 0b0000010;
  uint8_t current_gen_config_state = 0b0;

  // Write to gen config register to start a self cal
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_set_bit);
  Wire.write(adc_gen_config);
  Wire.write(self_cal_mask);
  Wire.endTransmission();

  // Begin checking if the self cal is complete
  while (!cal_complete) {
    // Read current self cal bit state
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_read);
    Wire.write(adc_gen_config);
    Wire.endTransmission();

    // Begin read
    Wire.requestFrom(adc_add, 1);
    current_gen_config_state = Wire.read();
    Wire.endTransmission();

    // Bitwise AND the current self cal bit with the self cal mask
    current_gen_config_state &= self_cal_mask;

    if (current_gen_config_state == 0) {
      cal_complete = true;
    }
  }

  debug_msg(DEBUG_PARTAL_ADC, "self cal complete", false, 0);

  return;
}

/**
 * @brief Initializes the ADC module with the specified configuration.
 * 
 * This function initializes the ADC module by configuring various parameters such as
 * hysteresis, thresholds, event counts, and pin modes. It also performs self-calibration
 * and sets up oversampling, mode, clock, sequence, and alert channel configurations.
 * 
 * @param fast_mode Flag indicating whether to enable fast mode or not.
 *                  If set to true, the ADC will operate in i2c fast mode with a higher clock speed for sending config information.
 *                  If set to false, the ADC will operate in i2c normal mode with a lower clock speed for sending config information.
 * 
 */
void adc_init(bool fast_mode) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_init called, beiging setup, fast mode=", true, fast_mode);

  // Initialize local variables
  uint8_t ch0_hysteresis = 0b0;
  uint8_t ch0_high_threshold = 0b0;
  uint8_t ch0_event_count = 0b0;
  uint8_t ch0_low_threshold = 0b0;
  uint8_t ch1_hysteresis = 0b0;
  uint8_t ch1_high_threshold = 0b0;
  uint8_t ch1_event_count = 0b0;
  uint8_t ch1_low_threshold = 0b0;
  uint8_t ch2_hysteresis = 0b0;
  uint8_t ch2_high_threshold = 0b0;
  uint8_t ch2_event_count = 0b0;
  uint8_t ch2_low_threshold = 0b0;
  uint8_t ch3_hysteresis = 0b0;
  uint8_t ch3_high_threshold = 0b0;
  uint8_t ch3_event_count = 0b0;
  uint8_t ch3_low_threshold = 0b0;
  uint8_t ch4_hysteresis = 0b0;
  uint8_t ch4_high_threshold = 0b0;
  uint8_t ch4_event_count = 0b0;
  uint8_t ch4_low_threshold = 0b0;
  uint8_t ch5_hysteresis = 0b0;
  uint8_t ch5_high_threshold = 0b0;
  uint8_t ch5_event_count = 0b0;
  uint8_t ch5_low_threshold = 0b0;
  uint8_t ch6_hysteresis = 0b0;
  uint8_t ch6_high_threshold = 0b0;
  uint8_t ch6_event_count = 0b0;
  uint8_t ch6_low_threshold = 0b0;
  uint8_t ch7_hysteresis = 0b0;
  uint8_t ch7_high_threshold = 0b0;
  uint8_t ch7_event_count = 0b0;
  uint8_t ch7_low_threshold = 0b0;

  // Broadcast on I2C bus for ADC self-addressing
  Wire.beginTransmission(0x00);
  Wire.write(0x04);
  Wire.endTransmission();

  //curent adc address
  uint8_t adc_add = adc_1_add;

  for (int i = 0; i < 2; i++) {
    // Begin general config
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_gen_config);
    // ADC datasheet page 34
    // rms_en, crc_en, stats_en, dwc_en, cnvst, ch_rst, cal, rst
    Wire.write(0b00111100);
    Wire.endTransmission();

    // Self-calibrate the ADC
    adc_self_cal(adc_add);

    // Begin data config init
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_data_config);
    // ADC datasheet page 35
    // fix_pat, resv, appendstat_b1, append_stat_b0, resv, resv, resv, resv
    Wire.write(0b00000000);
    Wire.endTransmission();

    // Begin oversample setup
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_osr_config);
    // ADC datasheet page 35
    // resv, resv, resv, res, resv, osr_b2, osr_b1, osr_b0
    Wire.write(0b00000111); // 128 oversamples
    Wire.endTransmission();

    // Begin mode and clock setup
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_opmode_config);

    // ADC datasheet pages 35&36
    // conv_on_err, conv_mode_b1, conv_mode_b0, osc_sel, clk_div_b3, clk_div_b2, clk_div_b1, clk_div_b0
    Wire.write(0b01100001);
    Wire.endTransmission();

    // Set pin modes all to analog input
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_pin_config);

    // ADC datasheet pages 36
    // gpio_cfg_7, gpio_cfg_, gpio_cfg_5, gpio_cfg_4, gpio_cfg_3, gpio_cfg_2, gpio_cfg_1, gpio_cfg_0
    Wire.write(0b00000000); // all pins as analog inputs
    Wire.endTransmission();

    // Sequence setup
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_sequence_config);
    // ADC datasheet page 38
    // resv, resv, resv, seq_start, resv, resv, seq_mode_b1, seq_mode_b0
    Wire.write(0b00010001); // full sequence mode
    Wire.endTransmission();

    // add all analog inputs from the auto sequencer
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_auto_sequence_channel_sel);
    // ADC datasheet page 39
    // ain_7, ain_6, ain_5, ain_4, ain_3, ain_2, ain_1, ain_0
    Wire.write(0b111111111); // all channels enabled
    Wire.endTransmission();

    // Setup which channels will be allowed to assert the alert pin
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_alert_channel_sel);
    // ADC datasheet page 39
    // ch7, ch6, ch5, ch4, ch3, ch2, ch1, ch0
    Wire.write(0b11111111); //all channels enabled
    Wire.endTransmission();

    //setup other alert assert conditions 
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_alert_map);
    //adc datasheet page 40
    //resv, resv, resv, resv, resv, resv, alert_rms, alert_crc
    Wire.write(0b00000000);
    Wire.endTransmission();

    //setup alert pin config
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_alert_pin_config);
    //adc datasheet page 40
    //resv, resv, resv, resv, resv, alert_drive, alert_logic_b1, alert_logic_b0
    Wire.write(0b00000100); //push pull, active low
    Wire.endTransmission();

    //set digital window comparitor to alert on vlaues within the upper and lower limits
    Wire.beginTransmission(adc_add);
    Wire.write(adc_op_single_write);
    Wire.write(adc_event_rgn);
    //adc datasheet page 42
    //ch7, ch6, ch5, ch4, ch3, ch2, ch1, ch0
    Wire.write(0b11111111);
    Wire.endTransmission();

    // Begin configuring bytes to be sent for channel config
    // Ch0 bitwise operations
    ch0_hysteresis = (adc_ch0_hysteresis << 4) | (adc_ch0_high_threshold & 0x000F);
    ch0_high_threshold = (adc_ch0_high_threshold >> 4);
    ch0_event_count = ((adc_ch0_low_threshold & 0x00F0) >> 4) | (adc_ch0_event_count << 4);
    ch0_low_threshold = (adc_ch0_low_threshold >> 4);

    // Ch1 bitwise operations
    ch1_hysteresis = (adc_ch1_hysteresis << 4) | (adc_ch1_high_threshold & 0x000F);
    ch1_high_threshold = (adc_ch1_high_threshold >> 4);
    ch1_event_count = ((adc_ch1_low_threshold & 0x00F0) >> 4) | (adc_ch1_event_count << 4);
    ch1_low_threshold = (adc_ch1_low_threshold >> 4);

    // Ch2 bitwise operations
    ch2_hysteresis = (adc_ch2_hysteresis << 4) | (adc_ch2_high_threshold & 0x000F);
    ch2_high_threshold = (adc_ch2_high_threshold >> 4);
    ch2_event_count = ((adc_ch2_low_threshold & 0x00F0) >> 4) | (adc_ch2_event_count << 4);
    ch2_low_threshold = (adc_ch2_low_threshold >> 4);

    // Ch3 bitwise operations
    ch3_hysteresis = (adc_ch3_hysteresis << 4) | (adc_ch3_high_threshold & 0x000F);
    ch3_high_threshold = (adc_ch3_high_threshold >> 4);
    ch3_event_count = ((adc_ch3_low_threshold & 0x00F0) >> 4) | (adc_ch3_event_count << 4);
    ch3_low_threshold = (adc_ch3_low_threshold >> 4);

    // Ch4 bitwise operations
    ch4_hysteresis = (adc_ch4_hysteresis << 4) | (adc_ch4_high_threshold & 0x000F);
    ch4_high_threshold = (adc_ch4_high_threshold >> 4);
    ch4_event_count = ((adc_ch4_low_threshold & 0x00F0) >> 4) | (adc_ch4_event_count << 4);
    ch4_low_threshold = (adc_ch4_low_threshold >> 4);

    // Ch5 bitwise operations
    ch5_hysteresis = (adc_ch5_hysteresis << 4) | (adc_ch5_high_threshold & 0x000F);
    ch5_high_threshold = (adc_ch5_high_threshold >> 4);
    ch5_event_count = ((adc_ch5_low_threshold & 0x00F0) >> 4) | (adc_ch5_event_count << 4);
    ch5_low_threshold = (adc_ch5_low_threshold >> 4);

    // Ch6 bitwise operations
    ch6_hysteresis = (adc_ch6_hysteresis << 4) | (adc_ch6_high_threshold & 0x000F);
    ch6_high_threshold = (adc_ch6_high_threshold >> 4);
    ch6_event_count = ((adc_ch6_low_threshold & 0x00F0) >> 4) | (adc_ch6_event_count << 4);
    ch6_low_threshold = (adc_ch6_low_threshold >> 4);

    // Ch7 bitwise operations
    ch7_hysteresis = (adc_ch7_hysteresis << 4) | (adc_ch7_high_threshold & 0x000F);
    ch7_high_threshold = (adc_ch7_high_threshold >> 4);
    ch7_event_count = ((adc_ch7_low_threshold & 0x00F0) >> 4) | (adc_ch7_event_count << 4);
    ch7_low_threshold = (adc_ch7_low_threshold >> 4);

    // Begin sending all the config data to the ADC
    Wire.beginTransmission(adc_add);
    if (fast_mode) { // Jump to fast mode if enabled
      Wire.write(0xB0);
      Wire.setClock(3400000);
    }
    Wire.write(adc_op_continuous_write);
    Wire.write(adc_hysteresis_ch0);
    Wire.write(ch0_hysteresis);
    Wire.write(ch0_high_threshold);
    Wire.write(ch0_event_count);
    Wire.write(ch0_low_threshold);
    Wire.write(ch1_hysteresis);
    Wire.write(ch1_high_threshold);
    Wire.write(ch1_event_count);
    Wire.write(ch1_low_threshold);
    Wire.write(ch2_hysteresis);
    Wire.write(ch2_high_threshold);
    Wire.write(ch2_event_count);
    Wire.write(ch2_low_threshold);
    Wire.write(ch3_hysteresis);
    Wire.write(ch3_high_threshold);
    Wire.write(ch3_event_count);
    Wire.write(ch3_low_threshold);
    Wire.write(ch4_hysteresis);
    Wire.write(ch4_high_threshold);
    Wire.write(ch4_event_count);
    Wire.write(ch4_low_threshold);
    Wire.write(ch5_hysteresis);
    Wire.write(ch5_high_threshold);
    Wire.write(ch5_event_count);
    Wire.write(ch5_low_threshold);
    Wire.write(ch6_hysteresis);
    Wire.write(ch6_high_threshold);
    Wire.write(ch6_event_count);
    Wire.write(ch6_low_threshold);
    Wire.write(ch7_hysteresis);
    Wire.write(ch7_high_threshold);
    Wire.write(ch7_event_count);
    Wire.write(ch7_low_threshold);
    Wire.endTransmission();
    if (fast_mode) {
      Wire.setClock(400000);
    }

    // Set the ADC address for the next iteration
    if (adc_add == adc_1_add) {
      adc_add = adc_2_add; // Switch to the second ADC address
    }
  }

  debug_msg(DEBUG_PARTAL_ADC, "adc init complete", false, 0);

  return;
}


/**
 * @brief Sets the high and low threshold values for a specific ADC channel.
 *
 * This function configures the threshold values for a given ADC channel, including hysteresis and event count,
 * by communicating with the ADC over I2C. It reads the current hysteresis and event count values, combines them
 * with the new threshold values, and writes the updated configuration back to the ADC.
 *
 * @param pin_needed   Structure containing ADC number and channel information.
 * @param high_th      The high threshold value to set (uint16_t).
 * @param low_th       The low threshold value to set (uint16_t).
 *
 * @note
 * - The function checks for valid ADC numbers (1 or 2) and valid channel numbers (0-7).
 * - If an invalid ADC number or channel is provided, the function logs an error and returns.
 * - The function uses the Wire library for I2C communication.
 * - Debug messages are logged at various stages for tracing and debugging.
 */
void adc_threshold_set(struct pin pin_needed , uint16_t high_th, uint16_t low_th) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called", false, 0);

  //init local vars
  uint8_t chx_hysteresis = 0b0;
  uint8_t chx_high_th = 0b0;
  uint8_t chx_event_count = 0b0;
  uint8_t chx_low_th = 0b0;
  uint8_t current_hysteresis_val = 0b0;
  uint8_t current_event_count = 0b0;
  uint8_t adc_add = 0b0; // ADC address to be set based on adc_num

  //set waht adc address to use based on adc_num
  if (pin_needed.adc_num == 1) {
    adc_add = adc_1_add; // Set the ADC address to the first ADC
  } else if (pin_needed.adc_num == 2) {
    adc_add = adc_2_add; // Set the ADC address to the second ADC
  } else {
    debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_num", false, pin_needed.adc_num);
    return; // Invalid ADC number, exit the function
  }

  //read current hysteresis & event count values
  //begin hysteresis read
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_continuous_read);

  switch (pin_needed.adc_channel){
    case 0:
      Wire.write(adc_ch0_hysteresis);
      break;
    case 1:
      Wire.write(adc_ch1_hysteresis);
      break;
    case 2:
      Wire.write(adc_ch2_hysteresis);
      break;
    case 3:
      Wire.write(adc_ch3_hysteresis);
      break;
    case 4:
      Wire.write(adc_ch4_hysteresis);
      break;
    case 5:
      Wire.write(adc_ch5_hysteresis);
      break;
    case 6:
      Wire.write(adc_ch6_hysteresis);
      break;
    case 7:
      Wire.write(adc_ch7_hysteresis);
      break;
    default:
      debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_channel", false, pin_needed.adc_channel);
    break; // Invalid ADC channel, exit the function
  }
  Wire.endTransmission();

  Wire.requestFrom(adc_add, 1);
  current_hysteresis_val = (Wire.read() & 0x0F);
  current_event_count = (Wire.read() & 0x0F);
  Wire.endTransmission();
  


  //convert values to format the adc accepts 
  chx_hysteresis = (((high_th & 0x000F) << 4) | current_hysteresis_val);
  chx_high_th = (high_th >> 4);
  chx_event_count = (((low_th & 0x000F) << 4) | current_event_count);
  chx_low_th = (low_th >> 4);

  // begin writeing to adc
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_continuous_write);

  switch (pin_needed.adc_channel){
    case 0:
      Wire.write(adc_ch0_hysteresis);
      break;
    case 1:
      Wire.write(adc_ch1_hysteresis);
      break;
    case 2:
      Wire.write(adc_ch2_hysteresis);
      break;
    case 3:
      Wire.write(adc_ch3_hysteresis);
      break;
    case 4:
      Wire.write(adc_ch4_hysteresis);
      break;
    case 5:
      Wire.write(adc_ch5_hysteresis);
      break;
    case 6:
      Wire.write(adc_ch6_hysteresis);
      break;
    case 7:
      Wire.write(adc_ch7_hysteresis);
      break;
    default:
      debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_channel", false, pin_needed.adc_channel);
    break; // Invalid ADC channel, exit the function
  }
  Wire.write(chx_hysteresis);
  Wire.write(chx_high_th);
  Wire.write(chx_event_count);
  Wire.write(chx_low_th);
  Wire.endTransmission();

  debug_msg(DEBUG_PARTAL_ADC, "threshold set complete", false, 0);
  debug_msg(DEBUG_PARTAL_ADC, "high threshold set to", true, high_th);
  debug_msg(DEBUG_PARTAL_ADC, "low threshold set to", true, low_th);
  debug_msg(DEBUG_PARTAL_ADC, "on adc:", true, pin_needed.adc_num);
  debug_msg(DEBUG_PARTAL_ADC, "on channel:", true, pin_needed.adc_channel);

  return;

}

/**
 * @brief Sets the hysteresis value for a specific ADC channel on a given ADC device.
 *
 * This function reads the current hysteresis register value for the specified ADC channel,
 * updates the hysteresis setting with the provided value, and writes it back to the ADC.
 * The function supports multiple ADC devices and channels, and communicates via the I2C (Wire) interface.
 *
 * @param pin_needed A struct containing information about the target ADC device and channel.
 *                   - pin_needed.adc_num: The ADC device number (1 or 2).
 *                   - pin_needed.adc_channel: The ADC channel number (0-7).
 * @param hysteresis_set The new hysteresis value to set (lower 4 bits are used).
 *
 * @note The function performs basic validation on the ADC channel number and logs debug messages.
 *       It assumes that the I2C Wire library is initialized and that the ADC address and register
 *       constants (e.g., adc_1_add, adc_ch0_hysteresis) are defined elsewhere.
 */
void adc_hysteresis_set(struct pin pin_needed, uint8_t hysteresis_set) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_histerisis_set called", false, 0);

  //init local vars
  uint8_t current_hysteresis_reg_val = 0x00;
  uint8_t chx_hysteresis = 0x00;
  uint8_t adc_add = 0b0; // ADC address to be set based on adc_num

  //set waht adc address to use based on pin_needed.adc_num
  if (pin_needed.adc_num == 1) {
    adc_add = adc_1_add; // Set the ADC address to the first ADC
  } else if (pin_needed.adc_num == 2) {
    adc_add = adc_2_add; // Set the ADC address to the second ADC
  }

  //read current hysteresis register value
  //begin hysteresis read
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_single_read);

  switch (pin_needed.adc_channel){
    case 0:
      Wire.write(adc_ch0_hysteresis);
      break;
    case 1:
      Wire.write(adc_ch1_hysteresis);
      break;
    case 2:
      Wire.write(adc_ch2_hysteresis);
      break;
    case 3:
      Wire.write(adc_ch3_hysteresis);
      break;
    case 4:
      Wire.write(adc_ch4_hysteresis);
      break;
    case 5:
      Wire.write(adc_ch5_hysteresis);
      break;
    case 6:
      Wire.write(adc_ch6_hysteresis);
      break;
    case 7:
      Wire.write(adc_ch7_hysteresis);
      break;
    default:
      debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_channel", false, pin_needed.adc_channel);
    break; // Invalid ADC channel, exit the function
  }

  Wire.endTransmission();

  Wire.requestFrom(adc_add, 1);
  current_hysteresis_reg_val = (Wire.read() & 0x0F);
  Wire.endTransmission();
  
  //convert values to format the adc accepts 
  chx_hysteresis = (((hysteresis_set & 0x000F) << 4) | current_hysteresis_reg_val);

  // begin writeing to adc
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_single_write);

  switch (pin_needed.adc_channel){
    case 0:
      Wire.write(adc_ch0_hysteresis);
      break;
    case 1:
      Wire.write(adc_ch1_hysteresis);
      break;
    case 2:
      Wire.write(adc_ch2_hysteresis);
      break;
    case 3:
      Wire.write(adc_ch3_hysteresis);
      break;
    case 4:
      Wire.write(adc_ch4_hysteresis);
      break;
    case 5:
      Wire.write(adc_ch5_hysteresis);
      break;
    case 6:
      Wire.write(adc_ch6_hysteresis);
      break;
    case 7:
      Wire.write(adc_ch7_hysteresis);
      break;
    default:
      debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_channel", false, pin_needed.adc_channel);
    break; // Invalid ADC channel, exit the function
  }
  
  Wire.write(chx_hysteresis);
  Wire.endTransmission();

  debug_msg(DEBUG_PARTAL_ADC, "histarisis set complete", false, 0);

  return;

}


/**
 * @brief Reads the event count for a specific ADC channel on a given ADC device.
 *
 * This function communicates with an external ADC via I2C to read the event count
 * (typically a 4-bit value) associated with the hysteresis register of the specified channel.
 * The ADC device is selected based on the adc_num field of the provided pin structure.
 * The channel is selected based on the adc_channel field.
 *
 * @param pin_needed A struct pin specifying the ADC device (adc_num) and channel (adc_channel) to read from.
 *                   - adc_num: The ADC device number (1 or 2).
 *                   - adc_channel: The ADC channel number (0-7).
 *
 * @return int The current event count (0-15) for the specified channel, or
 *             -1 if an invalid ADC number is provided.
 *
 * @note If an invalid channel is specified, the function logs a debug message but continues execution.
 *       If an invalid ADC number is specified, the function logs a debug message and returns -1.
 */
int adc_event_count_read(struct pin pin_needed) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_event_count_read called", false, 0);

  //init local vars 
  uint8_t chx_current_event_count = 0x00;
  uint8_t adc_add = 0b0; // ADC address to be set based on adc_num

  if (pin_needed.adc_num == 1) {
    adc_add = adc_1_add; // Set the ADC address to the first ADC
  } else if (pin_needed.adc_num == 2) {
    adc_add = adc_2_add; // Set the ADC address to the second ADC
  } else {
    debug_msg(DEBUG_PARTAL_ADC, "adc_event_count_read called with invalid adc_num", false, pin_needed.adc_num);
    return -1; // Invalid ADC number, exit the function
  }

  //begin event count read
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_single_read);
  switch (pin_needed.adc_channel){
    case 0:
      Wire.write(adc_event_count_ch0);
      break;
    case 1:
      Wire.write(adc_event_count_ch1);
      break;
    case 2:
      Wire.write(adc_event_count_ch2);
      break;
    case 3:
      Wire.write(adc_event_count_ch3);
      break;
    case 4:
      Wire.write(adc_event_count_ch4);
      break;
    case 5:
      Wire.write(adc_event_count_ch5);
      break;
    case 6:
      Wire.write(adc_event_count_ch6);
      break;
    case 7:
      Wire.write(adc_event_count_ch7);
      break;
    default:
      debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_channel", false, pin_needed.adc_channel);
    break; // Invalid ADC channel, exit the function
  }
  Wire.endTransmission();

  Wire.requestFrom(adc_add, 1);
  chx_current_event_count = (Wire.read() & 0x0F);

  debug_msg(DEBUG_PARTAL_ADC, "event count read complete with value", true, chx_current_event_count);

  return chx_current_event_count;
}

/**
 * @brief Clears the ADC event count register for a specified pin.
 *
 * This function resets the event count register (hysteresis register) of the ADC channel
 * associated with the provided pin. It determines the correct ADC device and channel,
 * then writes zero to the corresponding event count register via I2C.
 *
 * @param pin_needed The pin structure specifying the ADC number and channel to clear.
 *
 * @note If an invalid ADC number or channel is provided, the function logs an error and returns without making changes.
 */
void IRAM_ATTR adc_event_clear(struct pin pin_needed) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_event_clear called", false, 0);

  //init local vars
  uint8_t event_count_value = 0x00;
  uint8_t adc_add = 0b0; // ADC address to be set based on adc_num

  if (pin_needed.adc_num == 1) {
    adc_add = adc_1_add; // Set the ADC address to the first ADC
  } else if (pin_needed.adc_num == 2) {
    adc_add = adc_2_add; // Set the ADC address to the second ADC
  } else {
    debug_msg(DEBUG_PARTAL_ADC, "adc_event_clear called with invalid adc_num", false, pin_needed.adc_num);
    return; // Invalid ADC number, exit the function
  }

  //write 0 to event count register
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_single_write);
  switch (pin_needed.adc_channel){
    case 0:
      Wire.write(adc_event_count_ch0);
      break;
    case 1:
      Wire.write(adc_event_count_ch1);
      break;
    case 2:
      Wire.write(adc_event_count_ch2);
      break;
    case 3:
      Wire.write(adc_event_count_ch3);
      break;
    case 4:
      Wire.write(adc_event_count_ch4);
      break;
    case 5:
      Wire.write(adc_event_count_ch5);
      break;
    case 6:
      Wire.write(adc_event_count_ch6);
      break;
    case 7:
      Wire.write(adc_event_count_ch7);
      break;
    default:
      debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_channel", false, pin_needed.adc_channel);
    break; // Invalid ADC channel, exit the function
  }
  Wire.write(event_count_value);
  Wire.endTransmission();

  debug_msg(DEBUG_PARTAL_ADC, "event cleard", false, 0);
  
  return;

}

/**
 * @brief Reads the ADC value from the specified pin.
 *
 * This function communicates with an external ADC (Analog-to-Digital Converter)
 * over I2C to read the value from a specific channel and ADC number as specified
 * in the provided pin structure. It handles the selection of the correct ADC device
 * and channel, performs the I2C transaction, and returns the read value.
 *
 * @param pin_needed A struct containing information about the ADC number and channel to read from.
 *                   - pin_needed.adc_num: The ADC device number (1 or 2).
 *                   - pin_needed.adc_channel: The ADC channel number (0-7).
 *
 * @return int The 16-bit ADC value read from the specified channel, or -1 if an invalid ADC number is provided.
 *
 * @note The function uses debug messages for tracing and error reporting.
 * @note Returns -1 if an invalid ADC number is provided.
 * @note If an invalid channel is provided, a debug message is logged, but the function continues.
 */
int adc_read(struct pin pin_needed) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_read called", false, 0);

  //init local vars
  uint16_t read_word = 0x0000;
  uint8_t adc_add = 0b0; // ADC address to be set based on adc_num

  if (pin_needed.adc_num == 1) {
    adc_add = adc_1_add; // Set the ADC address to the first ADC
  } else if (pin_needed.adc_num == 2) {
    adc_add = adc_2_add; // Set the ADC address to the second ADC
  } else {
    debug_msg(DEBUG_PARTAL_ADC, "adc_read called with invalid adc_num", false, pin_needed.adc_num);
    return -1; // Invalid ADC number, exit the function
  }

  //begin read
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_continuous_read);

  switch (pin_needed.adc_channel){
    case 0:
      Wire.write(adc_recent_ch0_lsb);
      break;
    case 1:
      Wire.write(adc_recent_ch1_lsb);
      break;
    case 2:
      Wire.write(adc_recent_ch2_lsb);
      break;
    case 3:
      Wire.write(adc_recent_ch3_lsb);
      break;
    case 4:
      Wire.write(adc_recent_ch4_lsb);
      break;
    case 5:
      Wire.write(adc_recent_ch5_lsb);
      break;
    case 6:
      Wire.write(adc_recent_ch6_lsb);
      break;
    case 7:
      Wire.write(adc_recent_ch7_lsb);
      break;
    default:
      debug_msg(DEBUG_PARTAL_ADC, "adc_threshold_set called with invalid adc_channel", false, pin_needed.adc_channel);
    break; // Invalid ADC channel, exit the function
  }
  
  Wire.endTransmission();

  Wire.requestFrom(adc_add, 2);
  read_word = Wire.read();
  read_word |= (Wire.read() << 8); // Read the MSB and combine with LSB
  Wire.endTransmission();

  debug_msg(DEBUG_PARTAL_ADC, "adc read complete, with count", true, read_word);

  return read_word;

}

/**
 * @brief Clears the event flags for the specified ADC.
 *
 * This function sends a command over I2C to clear the event flags of the ADC
 * identified by the given adc_num. It selects the appropriate ADC address based
 * on the adc_num parameter, and writes the necessary sequence to clear the event
 * flags register. If an invalid adc_num is provided, the function logs a debug
 * message and returns without performing any operation.
 *
 * @param adc_num The ADC number to clear event flags for (1 or 2).
 *
 * @note If adc_num is not 1 or 2, the function will log an error and exit.
 */
void adc_clear_event_flags(int adc_num) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_clear_event_flags called", false, 0);

  //init local vars
  uint8_t adc_add = 0b0; // ADC address to be set based on adc_num

  if (adc_num == 1) {
    adc_add = adc_1_add; // Set the ADC address to the first ADC
  } else if (adc_num == 2) {
    adc_add = adc_2_add; // Set the ADC address to the second ADC
  } else {
    debug_msg(DEBUG_PARTAL_ADC, "adc_clear_event_flags called with invalid adc_num", false, adc_num);
    return; // Invalid ADC number, exit the function
  }

  Wire.write(adc_add);
  Wire.write(adc_op_single_write);
  Wire.write(adc_event_flag);
  Wire.write(0x0);
  Wire.endTransmission();
}

/**
 * @brief Determines the source pin that triggered an ADC alert.
 *
 * This function reads the event flag register from the specified ADC (based on the adc_num in the input pin structure)
 * to identify which channel caused the alert. It then searches through the list of pin structures to find and return
 * the pin corresponding to the triggered channel.
 *
 * @param alert_pin The pin structure containing at least the adc_num field to specify which ADC to query.
 * @return struct pin The pin structure corresponding to the channel that triggered the alert.
 *         If no valid source is found or adc_num is invalid, the function returns without a value.
 *
 * @note The function logs debug messages for tracing and error handling.
 */
struct pin adc_determine_alert_source(struct pin alert_pin) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_determine_alert_source called", false, 0);

  //init local vars 
  uint8_t current_event_flag_register_value = 0;
  uint8_t adc_add = 0b0; // ADC address to be set based on adc_num
  uint8_t test_mask = 0x01;
  int chanel_index = 0;

  if (alert_pin.adc_num == 1) {
    adc_add = adc_1_add; // Set the ADC address to the first ADC
  } else if (alert_pin.adc_num == 2) {
    adc_add = adc_2_add; // Set the ADC address to the second ADC
  } else {
    debug_msg(DEBUG_PARTAL_ADC, "adc_determine_alert_source called with invalid adc_num", false, alert_pin.adc_num);
  }

  //read alert flag register
  Wire.beginTransmission(adc_add);
  Wire.write(adc_op_single_read);
  Wire.write(adc_event_flag);
  Wire.endTransmission();
  Wire.requestFrom(adc_add, 1);
  current_event_flag_register_value = Wire.read();
  Wire.endTransmission();

  //determine what chanel triggered the alert
  for (int x; x <= 7; x++) {
    if (current_event_flag_register_value & test_mask) {
      chanel_index = x + 1; // Get the index of the channel that triggered the alert
    }
    test_mask <<= 1; // Shift the mask to check the next bit

  }

  for (size_t i = 1; i < sizeof(pin_names[0]); i++) {
    if (pin_names[i]->pin_mode == analog_in) {
      if (pin_names[i]->adc_num == alert_pin.adc_num) {
        if (chanel_index == pin_names[i]->adc_channel) {
          debug_msg(DEBUG_PARTAL_ADC, "adc_determine_alert_source complete", false, 0);
          return *pin_names[i]; // Return the pin structure for the channel that triggered the alert

        }
      }

    }
  }

}

/**
 * @brief Sets the current limit for a specified ADC channel.
 *
 * This function sets the current limit for the given pin/channel. If the pin has an associated
 * current limit pointer, the value is updated directly. For non-onboard pins, the function
 * calculates the equivalent 12-bit ADC count for the specified current limit (in mA) and sets
 * the ADC threshold accordingly.
 *
 * @param pin_needed The pin structure specifying the ADC channel and associated properties.
 * @param current_limit The desired current limit in milliamperes (mA).
 */
void adc_set_ch_current_limit(struct pin pin_needed, uint current_limit) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_set_ch_current_limit called", false, 0);

  if (pin_needed.chanel_curent_limit_pointer != nullptr) {
    *(pin_needed.chanel_curent_limit_pointer) = current_limit; // Set the value pointed to by the pointer
  }

  if (!pin_needed.onboard) {
    //calculate the current limit
    //calculate voltage drop across the shunt resistor
    float voltage_drop = (current_limit / 1000.0) * adc_current_sense_resistance; // Convert mA to A and multiply by resistance to get voltage

    //calculate the minimum voltage for csn
    float csn_voltage_min = ((adc_read(ref_12v) * (3.3 / 65535.0)) * 5) - voltage_drop; // Read the ADC value and convert to voltage (assuming 3.3V reference)

    //calculate the minimum voltage for csn in counts
    int csn_counts_min = (int)(((csn_voltage_min / 5) / 3.3) * 4095.0); // Convert voltage to counts (assuming 12-bit thresholds)
 
    //set threshold value for the channel
    adc_threshold_set(pin_needed, 0xFFF, csn_counts_min); // Set the low threshold with the calculated counts and a high threshold of 0
  }

  
  return;
}

/**
 * @brief Reads the current flowing through a specified channel using ADC measurements.
 *
 * This function measures the voltage drop across a shunt resistor and MOSFET to calculate
 * the current in milliamperes (mA) for the given pin. It reads the supply voltage and the
 * current sense (CSN) voltage, computes the voltage drop, and then calculates the current
 * using Ohm's law.
 *
 * @param pin_needed The pin structure specifying the pin number and whether it is onboard.
 * @return int The calculated current in milliamperes (mA).
 *
 * @note Assumes a 3.3V ADC reference and a 16-bit ADC resolution.
 * @note Uses the global variable 'adc_current_sense_resistance' for the shunt resistor value.
 */
int IRAM_ATTR adc_read_ch_current (struct pin pin_needed) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_read_ch_current called", false, 0);

  //curent csn value
  uint16_t current_csn_voltage_counts = 0;
  float current_csn_voltage = 0;
  float csn_min_voltage = 0;
  uint16_t current_supply_voltage_counts = 0;
  float current_supply_voltage = 0;
  float v_drop = 0;
  int current_mA = 0;

  //read the curent supply voltage
  current_supply_voltage_counts = (analogRead(pin_needed.pin_number)) << 4; // Read the analog value from the pin and map it to 16 bit

  //convert the counts to voltage
  current_supply_voltage = (((float)current_supply_voltage_counts / 65535.0) * 3.3) * 5; // Convert counts to voltage (assuming 3.3V reference)
  
  //read the curent csn value
  if (pin_needed.onboard) {
    current_csn_voltage_counts = (analogRead(pin_needed.pin_number)) << 4; // Read the analog value from the pin and map it to 16 bit
  } else {
    current_csn_voltage_counts = adc_read(pin_needed); // Read the ADC value from the specified pin
  }

  //convert the counts to voltage
  current_csn_voltage = (((float)current_csn_voltage_counts / 65535.0) * 3.3) * 5; // Convert counts to voltage (assuming 3.3V reference)
  current_supply_voltage = (((float)current_supply_voltage_counts / 65535.0) * 3.3) * 5; // Convert counts to voltage (assuming 3.3V reference)

  //subtract the csn voltage from the supply voltage to get the voltage drop across the shunt resistor & mosfet
  v_drop = current_supply_voltage - current_csn_voltage;

  //calculate the current in mA using the formula: I = V / R
  current_mA = (int)((v_drop / adc_current_sense_resistance) * 1000.0); // Convert voltage drop to current in mA

  debug_msg(DEBUG_PARTAL_ADC, "adc_read_ch_current complete with value", true, current_mA);

  return current_mA; // Return the calculated current in mA

}

/**
 * @brief Handles ADC alert events triggered by a specified alert pin.
 *
 * This function is called when an ADC alert is detected. It determines the source of the alert,
 * finds the corresponding gate pin, checks if the alert source is in an overcurrent state,
 * and takes appropriate action (such as disabling the gate pin and clearing the alert).
 * Debug messages are logged throughout the process for diagnostic purposes.
 *
 * @param alert_pin The pin structure representing the alert pin that triggered the handler.
 *
 * @note This function assumes that the pin_names array and related structures are properly initialized.
 *       It also assumes that the alert source's current limit pointer is valid.
 */
void IRAM_ATTR adc_alert_handler (struct pin alert_pin) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_alert_handler called", false, 0);

  //determine the source of the alert
  struct pin alert_source = adc_determine_alert_source(alert_pin);
  debug_msg(DEBUG_PARTAL_ADC, "alert source found on pin:", true, alert_source.pin_number);
  debug_msg(DEBUG_PARTAL_ADC, "on adc:", true, alert_source.adc_num);
  debug_msg(DEBUG_PARTAL_ADC, "on channel:", true, alert_source.adc_channel);

  //find gate pin for the alert source
  struct pin gate_pin; // Declare gate_pin before the loop
  for (size_t i = 1; i < sizeof(pin_names[0]); i++) {
    if (pin_names[i]->pin_mode == out){
      if (pin_names[i]->adc_num == alert_source.adc_num) {
        if (pin_names[i]->adc_channel == alert_source.adc_channel) {
          debug_msg(DEBUG_PARTAL_ADC, "gate pin found on pin:", true, pin_names[i]->pin_number);
          gate_pin = *pin_names[i]; // Copy the gate pin structure
          break; // Exit the loop once the gate pin is found
        }
      }
    }
  }
    
  //check if the alert soucrce is in over current state
  if (adc_read_ch_current(alert_source) > *alert_source.chanel_curent_limit_pointer) {
    debug_msg(DEBUG_PARTAL_ADC, "current limit exceeded on alerting channel", false, 0);
    io_call(gate_pin, WRITE, low);
    adc_event_clear(alert_source); // Clear the alert for the source channel
      
  } else {
    debug_msg(DEBUG_PARTAL_ADC, "current limit not exceeded on alert source", false, 0);
  }

  //call rtos follow up function


  return;
}

/**
 * @brief Checks the current on a peripheral power channel and disables the channel if the current exceeds a predefined limit.
 *
 * This function reads the current value from the specified ADC pin associated with a peripheral power channel.
 * It compares the measured current (converted from ADC counts) to a user-defined current limit (in mA).
 * If the measured current exceeds the limit, the function disables the peripheral power channel by calling
 * the appropriate I/O function. Debug messages are logged throughout the process.
 *
 * No parameters or return value.
 */
void adc_monitor_periral_ch_current(void * parameter) {

  debug_msg(DEBUG_PARTAL_ADC, "adc_check_periral_ch_current called", false, 0);

  //vars
  float voltage_drop = 0.0;
  float reference_voltage = 0.0; // Reference voltage for the ADC
  float min_csn_voltage = 0.0; // Minimum CSN voltage for the current limit
  int current_csn_voltage_counts = 0; // Current CSN voltage in counts
  float ciurrent_csn_voltage = 0.0; // Current CSN voltage in volts

  //calculate the voltage drop across the shunt resistor and MOSFET
  voltage_drop = (*(peripheral_pwr_csn.chanel_curent_limit_pointer)) * adc_current_sense_resistance; // Calculate the voltage drop across the shunt resistor and MOSFET

  //read the reference voltage for the ADC
  reference_voltage = ((analogRead(peripheral_pwr_csn.pin_number) / 4095.0) * 3.3) * 5; // Convert the ADC reading to voltage (assuming 12-bit ADC and 3.3V reference)

  //calculate the min csn voltage for the current limit
  min_csn_voltage = reference_voltage - voltage_drop; // Calculate the minimum CSN voltage for the current limit



  for (;;) {
    //read csn voltage
    current_csn_voltage_counts = analogRead(peripheral_pwr_csn.pin_number); // Read the ADC value from the peripheral power channel
    ciurrent_csn_voltage = (((float)current_csn_voltage_counts / 4095.0) * 3.3) * 5; // Convert counts to voltage (assuming 12-bit ADC and 3.3V reference)

    if (ciurrent_csn_voltage < min_csn_voltage) {
      debug_msg(DEBUG_PARTAL_ADC, "current limit exceeded on peripheral power channel", false, 0);
      io_call(peripheral_pwr_csn, WRITE, low); // Disable the peripheral power channel

    } else {
      debug_msg(DEBUG_PARTAL_ADC, "current limit not exceeded on peripheral power channel", false, 0);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100 milliseconds before the next check

  }
  

  return;
}




#endif // adc_cpp