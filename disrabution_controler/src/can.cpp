#ifndef can_cpp
#define can_cpp

#include <Arduino.h>
#include "can.h"
#include "DEBUG.h"
#include "io.h"
#include "driver/twai.h"


/**
 * @brief Sends a CAN frame using the TWAI (CAN) driver.
 *
 * This function prepares a CAN frame based on the provided message structure
 * and attempts to transmit it immediately. If the transmit buffer is full,
 * it logs the event and (optionally) notifies a FreeRTOS task to retry sending.
 *
 * @param message_to_send The message structure containing CAN frame parameters:
 *   - id: CAN identifier.
 *   - extended: Whether to use extended frame format.
 *   - type: Frame type (e.g., data or RTR).
 *   - single_shot: Whether to use single shot transmission.
 *   - data: Array of data bytes to send.
 *
 * @note
 * - The function uses debug_msg for logging.
 * - If transmission fails due to a full buffer, notification to a sender task
 *   is possible (see commented code).
 * - Assumes the existence of a FreeRTOS task handle (canSenderTaskHandle) if
 *   notification is enabled.
 */
void can_send_frame(struct message message_to_send) {

  debug_msg(DEBUG_PARTIAL_CAN, "can_send_frame called", false, 0);

  //setup frame
  twai_message_t frame;
  frame.extd = message_to_send.extended; // Set extended frame format
  frame.rtr = (message_to_send.type == FRAME_RTR); // Set RTR flag if frame type is RTR
  frame.ss = message_to_send.single_shot; // Set single shot transmission flag
  frame.identifier = message_to_send.id; // Set the CAN ID
  frame.data_length_code = sizeof(message_to_send.data); // Set the data length code
  for (int i = 0; i < sizeof(message_to_send.data); i++) {
    frame.data[i] = message_to_send.data[i]; // Copy data bytes into the frame
  }

  // Try to send frame without blocking indefinitely
  if (twai_transmit(&frame, 0) == ESP_OK) {
    debug_msg(DEBUG_PARTIAL_CAN, "Frame sent successfully", false, 0);
  } else {
    debug_msg(DEBUG_PARTIAL_CAN, "Transmit buffer full, notifying sender task", false, 0);
    // Notify a FreeRTOS task to retry sending later
    // Assume you have a TaskHandle_t canSenderTaskHandle defined elsewhere
    /*
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(canSenderTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // Optionally, add the frame to a queue for the sender task to process\
    */
  }

  return;
}


/**
 * @brief Initializes the CAN (Controller Area Network) bus using the TWAI driver.
 *
 * This function sets up the CAN bus by configuring the general, timing, and filter parameters
 * using default macro initializers. It installs and starts the TWAI driver, and provides debug
 * messages at each step to indicate success or failure.
 *
 * Steps performed:
 * - Initializes configuration structures for general, timing, and filter settings.
 * - Installs the TWAI driver with the specified configurations.
 * - Starts the TWAI driver if installation is successful.
 * - Outputs debug messages for each major step and error condition.
 *
 * @note This function assumes that the `can_tx` and `can_rx` pin numbers are properly set.
 * @note Uses 500 Kbps as the CAN bus speed and accepts all CAN frames by default.
 */
void can_init() {
  debug_msg(DEBUG_PARTIAL_CAN, "can_init called, initializing CAN bus", false, 0);

  //enable can phy
  io_call(can_en, WRITE, high);
  io_call(can_stb, WRITE, high);
  debug_msg(DEBUG_PARTIAL_CAN, "CAN PHY enabled", false, 0);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)can_tx.pin_number,
   (gpio_num_t)can_rx.pin_number,
   TWAI_MODE_NORMAL);
   
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // install the TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    debug_msg(DEBUG_PARTIAL_CAN, "Failed to install TWAI driver", false, 0);
    return;
  } else {
    debug_msg(DEBUG_PARTIAL_CAN, "TWAI driver installed successfully", false, 0);
  }

  // Start the TWAI driver
  if (twai_start() != ESP_OK) {
    debug_msg(DEBUG_PARTIAL_CAN, "Failed to start TWAI driver", false, 0);
    return;
  } else {
    debug_msg(DEBUG_PARTIAL_CAN, "TWAI driver started successfully", false, 0);
  }

  // setup alerts
twai_reconfigure_alerts(
  TWAI_ALERT_TX_IDLE |
  TWAI_ALERT_TX_SUCCESS |
  TWAI_ALERT_RX_DATA |
  TWAI_ALERT_BUS_ERROR |
  TWAI_ALERT_TX_FAILED |
  TWAI_ALERT_RX_QUEUE_FULL,
  nullptr
);

  debug_msg(DEBUG_PARTIAL_CAN, "CAN bus initialized", false, 0);

  return;
}

/**
 * @brief Reads a received CAN frame from the TWAI bus.
 *
 * This function attempts to receive a CAN frame from the TWAI bus within a 1000 ms timeout.
 * If a frame is successfully received, it populates a `messaage` struct with the frame's
 * identifier, extended flag, frame type (data or remote), single shot flag, and data bytes.
 * Debug messages are logged for both successful and unsuccessful receive attempts.
 *
 * @return struct message The received CAN message. If no frame is received or an error occurs,
 *         the returned struct may contain uninitialized data.
 */
struct message can_read_recived_frame() {
  debug_msg(DEBUG_PARTIAL_CAN, "can_read_received_frame called", false, 0);

  twai_message_t frame;
  struct message recived_message;

  // Read a frame from the TWAI bus
  if (twai_receive(&frame, pdMS_TO_TICKS(1000)) == ESP_OK) {
    debug_msg(DEBUG_PARTIAL_CAN, "Frame received successfully", false, 0);
    recived_message.id = frame.identifier;
    recived_message.extended = frame.extd;
    recived_message.type = (frame.rtr) ? FRAME_RTR : FRAME_DATA;
    recived_message.single_shot = frame.ss;
    for (int i = 0; i < frame.data_length_code; i++) {
      recived_message.data[i] = frame.data[i];
    }
  } else {
    debug_msg(DEBUG_PARTIAL_CAN, "No frame received or error occurred", false, 0);
  }

  return recived_message;
}








#endif // can_cpp