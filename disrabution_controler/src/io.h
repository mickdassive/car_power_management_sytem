//{usbc car module}
//{io.h}
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

#ifndef io_h
#define io_h

#include <Arduino.h>
#include "adc.h"

//iox defines
const uint8_t iox_0_add = 0x20;
const uint8_t iox_input_port_0 = 0x00;
const uint8_t iox_input_port_1 = 0x01;
const uint8_t iox_output_port_0 = 0x02;
const uint8_t iox_output_port_1 = 0x03;
const uint8_t iox_pol_inv_port_0 = 0x04;
const uint8_t iox_pol_inv_port_1 = 0x05;
const uint8_t iox_config_port_0 = 0x06;
const uint8_t iox_config_port_1 = 0x07;
const uint8_t iox_drive_strength_register_00 = 0x40;
const uint8_t iox_drive_strenght_register_01 = 0x41;
const uint8_t iox_drive_strenght_register_10 = 0x42;
const uint8_t iox_drive_strenght_register_11 = 0x43;
const uint8_t iox_in_latch_register_0 = 0x44;
const uint8_t iox_in_ltach_register_1 =0x45;
const uint8_t iox_pull_up_down_en_register_0 = 0x46;
const uint8_t iox_pull_up_down_en_register_1 = 0x47;
const uint8_t iox_pull_up_down_sel_register_0 = 0x48;
const uint8_t iox_pull_up_down_sel_register_1 = 0x49;
const uint8_t iox_int_mask_register_0 = 0x4a;
const uint8_t iox_int_mask_register_1 = 0x4b;
const uint8_t iox_int_stat_register_0 = 0x4c;
const uint8_t iox_int_stat_register_1 = 0x4d;
const uint8_t iox_out_port_config_register = 0x4f;

//vars for chanel current limit in mA
extern uint16_t ch1_current_limit;
extern uint16_t ch2_current_limit;
extern uint16_t ch3_current_limit;
extern uint16_t ch4_current_limit;
extern uint16_t ch5_current_limit;
extern uint16_t ch6_current_limit;
extern uint16_t ch7_current_limit;
extern uint16_t ch8_current_limit;
extern uint16_t ch9_current_limit;
extern uint16_t ch10_current_limit;
extern uint16_t ch11_current_limit;
extern uint16_t ch12_current_limit;
extern uint16_t ch13_current_limit;
extern uint16_t ch14_current_limit;
extern uint16_t ch15_current_limit;
extern uint16_t ch16_current_limit;
extern uint16_t peripheral_current_limit;

//io_call struct setup
enum in_out {
  in,        //input
  out,       //output
  i2c,       //used for i2c pins only
  intr,      //used for software interupts only works with onboard pins
  empty_pin, //empty for empty pin object
  analog_in,  //used for analog pins
  analog_out, //used for analog output pins
  usb
};

struct pin {
  uint8_t mask;          //mask for iox
  int port;              //port on iox (can be 0 or 1 only)
                         //ECEPTION: i2c sda/scl defined by port 2 for sda and port 3 for scl
  int pin_number;        //pin number, for ESP-12F: GPIO#, for iox: phisical pin#
  int adc_num;           //adc number (can be 1 or 2 only)
  int adc_channel;       //adc channel (can be 0-7 only)
  in_out pin_mode;       //defines pinmode see enum above for deatails
  bool onboard;          //If true, IO is on ESP8266; if false, IO is on IO expander
  void (*interrupt_handler)(struct pin); // Pointer to interrupt handler function (optional)
  uint16_t* chanel_curent_limit_pointer;  // pointer to the current limit for this channel, if not set it will be nullptr


};
//BE SURE TO ADD PINS TO BOTH THE "pin" STRUCT AND THE "pin_names" ARRAY THEY MUST MATCH
//io_call struct defines  {mask, port, pin, pin_mode, onboard, interrupt_handler(optional)}
static struct pin sda {0x00, 2, 2, -1, -1, i2c, true};
static struct pin scl {0x00, 3, 3, -1, -1, i2c, true};
static struct pin iox_0_int {0x00, -1, 1, -1, -1, intr, true, nullptr};
static struct pin car_acc {0x00, -1, 4, -1, -1, in, true};
static struct pin peripheral_pwr_gate {0x00, -1, 5, -1, -1, out, true};
static struct pin gpio_6 {0x00, -1, 6, -1, -1, in, true};
static struct pin gpio_7 {0x00, -1, 7, -1, -1, in, true};
static struct pin gpio_8 {0x00, -1, 8, -1, -1, in, true};
static struct pin gpio_9 {0x00, -1, 9, -1, -1, in, true};
static struct pin gpio_10 {0x00, -1, 10, -1, -1, in, true};
static struct pin gpio_11 {0x00, -1, 11, -1, -1, in, true};
static struct pin gpio_12 {0x00, -1, 12, -1, -1, in, true};
static struct pin peripheral_pwr_csn {0x00, -1, 13, -1, -1, analog_in, true, nullptr, &peripheral_current_limit};
static struct pin ref_12v {0x00, -1, 14, -1, -1, analog_in, true};
static struct pin dac_1 {0x00, -1, 15, -1, -1, analog_out, true};
static struct pin dac_2 {0x00, -1, 16, -1, -1, analog_out, true};
static struct pin usb_d_n {0x00, -1, 19, -1, -1, usb, true};
static struct pin usb_d_p {0x00, -1, 20, -1, -1, usb, true};
static struct pin disrabution_neo_pixels {0x00, -1, 21, -1, -1, out, true};
static struct pin adc_1_alert {0x00, -1, 15, 1, -1, intr, true, adc_alert_handler};
static struct pin adc_2_alert {0x00, -1, 16, 2, -1, intr, true, adc_alert_handler};
static struct pin can_err {0x00, -1, 33, -1, -1, in, true};
static struct pin can_wake {0x00,-1, 34, -1, -1, intr, true};
static struct pin can_stb {0x00, -1, 35, -1, -1, out, true};
static struct pin can_tx {0x00, -1, 36, -1, -1, out, true};
static struct pin can_en {0x00, -1, 38, -1, -1, out, true};
static struct pin can_rx {0x00, -1, 37, -1, -1, in, true};
static struct pin ch1_gate {0x80, 0, 11, 1, 0, out, false};
static struct pin ch2_gate {0x40, 0, 10, 1, 1, out, false};
static struct pin ch3_gate {0x20, 0, 9, 1, 2, out, false};
static struct pin ch4_gate {0x10, 0, 8, 1, 3, out, false};
static struct pin ch5_gate {0x08, 0, 7, 1, 4, out, false};
static struct pin ch6_gate {0x04, 0, 6, 1, 5, out, false};
static struct pin ch7_gate {0x02, 0, 5, 1, 6, out, false};
static struct pin ch8_gate {0x01, 0, 4, 1, 7, out, false};
static struct pin ch9_gate {0x80, 1, 20, 2, 0, out, false};
static struct pin ch10_gate {0x40, 1, 19, 2, 1, out, false};
static struct pin ch11_gate {0x20, 1, 18, 2, 2, out, false};
static struct pin ch12_gate {0x10, 1, 17, 2, 3, out, false};
static struct pin ch13_gate {0x08, 1, 16, 2, 4, out, false};
static struct pin ch14_gate {0x04, 1, 15, 2, 5, out, false};
static struct pin ch15_gate {0x02, 1, 14, 2, 6, out, false};
static struct pin ch16_gate {0x01, 1, 13, 2, 7, out, false};
static struct pin ch1_csn {0x00, -1, 6, 1, 0, analog_in, false, nullptr, &ch1_current_limit};
static struct pin ch2_csn {0x00, -1, 5, 1, 1, analog_in, false, nullptr, &ch2_current_limit};
static struct pin ch3_csn {0x00, -1, 4, 1, 2, analog_in, false, nullptr, &ch3_current_limit};
static struct pin ch4_csn {0x00, -1, 3, 1, 3, analog_in, false, nullptr, &ch4_current_limit};
static struct pin ch5_csn {0x00, -1, 2, 1, 4, analog_in, false, nullptr, &ch5_current_limit};
static struct pin ch6_csn {0x00, -1, 1, 1, 5, analog_in, false, nullptr, &ch6_current_limit};
static struct pin ch7_csn {0x00, -1, 16, 1, 6, analog_in, false, nullptr, &ch7_current_limit};
static struct pin ch8_csn {0x00, -1, 15, 1, 7, analog_in, false, nullptr, &ch8_current_limit};
static struct pin ch9_csn {0x00, -1, 6, 2, 0, analog_in, false, nullptr, &ch9_current_limit};
static struct pin ch10_csn {0x00, -1, 5, 2, 1, analog_in, false, nullptr, &ch10_current_limit};
static struct pin ch11_csn {0x00, -1, 4, 2, 2, analog_in, false, nullptr, &ch11_current_limit};
static struct pin ch12_csn {0x00, -1, 3, 2, 3, analog_in, false, nullptr, &ch12_current_limit};
static struct pin ch13_csn {0x00, -1, 2, 2, 4, analog_in, false, nullptr, &ch13_current_limit};
static struct pin ch14_csn {0x00, -1, 1, 2, 5, analog_in, false, nullptr, &ch14_current_limit};
static struct pin ch15_csn {0x00, -1, 16, 2, 6, analog_in, false, nullptr, &ch15_current_limit};
static struct pin ch16_csn {0x00, -1, 15, 2, 7, analog_in, false, nullptr, &ch16_current_limit};

 

//Array of pin structs for automatic pin initialization
static struct pin* pin_names[] = {
  &sda,
  &scl,
  &iox_0_int,
  &car_acc,
  &peripheral_pwr_gate,
  &gpio_6,
  &gpio_7,
  &gpio_8,
  &gpio_9,
  &gpio_10,
  &gpio_11,
  &gpio_12,
  &peripheral_pwr_csn,
  &ref_12v,
  &dac_1,
  &dac_2,
  &usb_d_n,
  &usb_d_p,
  &disrabution_neo_pixels,
  &adc_1_alert,
  &adc_2_alert,
  &can_err,
  &can_wake,
  &can_stb,
  &can_tx,
  &can_en,
  &can_rx,
  &ch1_gate,
  &ch2_gate,
  &ch3_gate,
  &ch4_gate,
  &ch5_gate,
  &ch6_gate,
  &ch7_gate,
  &ch8_gate,
  &ch9_gate,
  &ch10_gate,
  &ch11_gate,
  &ch12_gate,
  &ch13_gate,
  &ch14_gate,
  &ch15_gate,
  &ch16_gate,
  &ch1_csn,
  &ch2_csn,
  &ch3_csn,
  &ch4_csn,
  &ch5_csn,
  &ch6_csn,
  &ch7_csn,
  &ch8_csn,
  &ch9_csn,
  &ch10_csn,
  &ch11_csn,
  &ch12_csn,
  &ch13_csn,
  &ch14_csn,
  &ch15_csn,
  &ch16_csn
};


//read write enum define for io_call function
enum read_write {
  read,
  write
};

//io_call high low input enum
enum high_low {
  high,
  low,
  read_mode
};

//function defines
uint8_t io_read_current_io_state(u_int8_t port, u_int8_t iox_num);
int io_call(struct pin pin_needed, enum read_write read_write, enum high_low high_low);
void io_gpio_init();




#endif // io_h
