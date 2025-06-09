#ifndef can_cpp
#define can_cpp

#include <Arduino.h>
#include "can.h"
#include "DEBUG.h"
#include "io.h"
#include "adc.h"
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

/**
 * @brief Task function to monitor and handle CAN (TWAI) controller alerts.
 *
 * This function continuously checks for various alerts from the CAN controller,
 * such as transmission success, transmission idle, reception of data, bus errors,
 * transmission failures, and receive queue overflows. For each detected alert,
 * it logs a corresponding debug message. The function is intended to run as a FreeRTOS task.
 *
 * @param pvParameters Pointer to task parameters (unused).
 *
 * Alerts handled:
 * - TWAI_ALERT_TX_IDLE: No more messages to send.
 * - TWAI_ALERT_TX_SUCCESS: CAN frame sent successfully.
 * - TWAI_ALERT_RX_DATA: Data received.
 * - TWAI_ALERT_BUS_ERROR: Bus error detected.
 * - TWAI_ALERT_TX_FAILED: Transmission failed.
 * - TWAI_ALERT_RX_QUEUE_FULL: Receive queue is full.
 *
 * The function includes a delay to avoid busy-waiting.
 */
void can_check_for_alerts (void *pvParameters) {

  debug_msg(DEBUG_PARTIAL_CAN, "can_check_for_alerts called", false, 0);

  uint32_t triggerd_alerts;

  for (;;) {

    twai_read_alerts(&triggerd_alerts, pdMS_TO_TICKS(100));

    if (triggerd_alerts & TWAI_ALERT_TX_IDLE) {

      debug_msg(DEBUG_PARTIAL_CAN, "TX Idle no more messages to send", false, 0);

    } else if (triggerd_alerts & TWAI_ALERT_TX_SUCCESS) {

      debug_msg(DEBUG_PARTIAL_CAN, "can frame sent sucessfuly", false, 0);

    } else if (triggerd_alerts & TWAI_ALERT_RX_DATA) {

      debug_msg(DEBUG_PARTIAL_CAN, "RX Data alert triggered", false, 0);

      // Read the received CAN frame
      struct message recived_message = can_read_recived_frame();

      // Allocate memory for the message to pass to the handler task
      struct message* recived_message_ptr = new struct message(recived_message);
      if (recived_message_ptr == nullptr) {
        debug_msg(DEBUG_PARTIAL_CAN, "can_message_received_handler_task: memory allocation failed", false, 0);
        vTaskDelete(NULL);
        return;
      }

      // Get the current task's priority to inherit
      UBaseType_t parent_priority = uxTaskPriorityGet(NULL);
      // Create the handler task
      xTaskCreate(
        can_message_received_handler_task, // Task function
        "can_message_received_handler",    // Name of the task
        2048,                             // Stack size in words
        recived_message_ptr,              // Task parameters (pointer to received message)
        parent_priority,                  // Task priority
        NULL                              // Task handle
      );

    } else if (triggerd_alerts & TWAI_ALERT_BUS_ERROR) {

      debug_msg(DEBUG_PARTIAL_CAN, "Bus Error alert triggered", false, 0);

    } else if (triggerd_alerts & TWAI_ALERT_TX_FAILED) {

      debug_msg(DEBUG_PARTIAL_CAN, "TX Failed alert triggered", false, 0);

    } else if (triggerd_alerts & TWAI_ALERT_RX_QUEUE_FULL) {

      debug_msg(DEBUG_PARTIAL_CAN, "RX Queue Full alert triggered", false, 0);
      
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to avoid busy-waiting

  }

}


/**
 * @brief RTOS task handler for processing received CAN messages.
 *
 * This function is designed to be run as a FreeRTOS task. It processes a received CAN message,
 * determines its type and intended recipient, and takes appropriate actions based on the message content.
 * The function supports handling both broadcast and direct messages for the distribution controller,
 * including heartbeat, fault, IO state, ADC read, status, channel current, set current limit, total current,
 * and channel control messages.
 *
 * The function expects a dynamically allocated pointer to a `struct message` as its parameter,
 * which it takes ownership of and deletes after copying.
 *
 * Actions performed by this handler include:
 * - Updating module heartbeat and fault status.
 * - Handling status flags such as "allow off-road" and "blackout".
 * - Creating RTOS tasks for ADC current limit and IO channel control operations.
 * - Logging debug messages for all significant events and message types.
 *
 * @param pvParameters Pointer to a dynamically allocated `struct message` containing the received CAN message.
 *                    The memory is freed within the function.
 *
 * @note This function should be used as a FreeRTOS task entry point.
 * @note The function assumes that all necessary global variables and structures (such as `can_modules`,
 *       `can_module_brodcast`, `can_module_disrabution_controler`, `pin_names`, etc.) are properly defined and initialized.
 */
void can_message_received_handler_task(void *pvParameters) {
  // Cast the parameter to a pointer to struct message
  struct message* received_message_ptr = (struct message*)pvParameters;
  if (received_message_ptr == nullptr) {
    debug_msg(DEBUG_PARTIAL_CAN, "can_message_received_handler_task: null parameter", false, 0);
    vTaskDelete(NULL);
    return;
  }
  struct message received_message = *received_message_ptr;
  // Free the parameter memory if it was dynamically allocated
  delete received_message_ptr;


  debug_msg(DEBUG_PARTIAL_CAN, "can_message_received_handler called", false, 0);

  debug_msg(DEBUG_PARTIAL_CAN, "Received message with ID:", true, received_message.id);
  debug_msg(DEBUG_PARTIAL_CAN, "Message type:", true, received_message.type);

  if (received_message.id == can_module_brodcast.bus_id) {

    // check if we received an RTR message
    if (received_message.type == FRAME_RTR) {
      debug_msg(DEBUG_PARTIAL_CAN, "Received a RTR message", false, 0);

      //call rtr handler and pass message

      // Allocate memory for the message to pass to the handler task
      struct message* rtr_message_ptr = new struct message(received_message);
      if (rtr_message_ptr == nullptr) {
        debug_msg(DEBUG_PARTIAL_CAN, "can_rtr_message_handler_taks: memory allocation failed", false, 0);
        vTaskDelete(NULL);
        return;
      }
      // Inherit the parent thread's priority
      UBaseType_t parent_priority = uxTaskPriorityGet(NULL);
      xTaskCreate(
        can_rtr_message_handler_taks, // Task function
        "can_rtr_handler",            // Name of the task
        2048,                        // Stack size in words
        rtr_message_ptr,             // Task parameters (pointer to received message)
        parent_priority,             // Task priority
        NULL                         // Task handle
      );

    } else {

      debug_msg(DEBUG_PARTIAL_CAN, "Received a data message", false, 0);

      // determine broadcast message type
      switch (received_message.data[0]) {
        case can_paket_type_heartbeat:
          debug_msg(DEBUG_PARTIAL_CAN, "Received a heartbeat message", false, 0);

          // determine what module the heartbeat is from
          for (size_t i = 1; i < sizeof(can_modules) / sizeof(can_modules[0]); i++) {
            if (can_modules[i]->bus_id == received_message.id) {
              can_modules[i]->last_heartbeat = millis();
              can_modules[i]->is_alive = true;
              debug_msg(DEBUG_PARTIAL_CAN, "Heartbeat from module with ID:", true, can_modules[i]->bus_id);
              break;
            }
          }
          break;
        case can_paket_type_module_fult:
          debug_msg(DEBUG_PARTIAL_CAN, "Received a module fault message. Module ID:", true, received_message.id);
          debug_msg(DEBUG_PARTIAL_CAN, "Fault status set to true for module ID:", true, received_message.id);

          // determine what module the fault is from
          for (size_t i = 1; i < sizeof(can_modules[0]); i++) {
            if (can_modules[i]->bus_id == received_message.id) {
              can_modules[i]->if_faulted = true;
              debug_msg(DEBUG_PARTIAL_CAN, "Fault from module with ID:", true, can_modules[i]->bus_id);
              break;
            }
          }
          break;
      }
    }
  } else if (received_message.id == can_module_disrabution_controler.bus_id) {
    debug_msg(DEBUG_PARTIAL_CAN, "Received a message for distribution controller", false, 0);

    // check if we received an RTR message
    if (received_message.type == FRAME_RTR) {
      debug_msg(DEBUG_PARTIAL_CAN, "Received a RTR message for distribution controller", false, 0);

      //call rtr handler and pass message

      // Allocate memory for the message to pass to the handler task
      struct message* rtr_message_ptr = new struct message(received_message);
      if (rtr_message_ptr == nullptr) {
        debug_msg(DEBUG_PARTIAL_CAN, "can_rtr_message_handler_taks: memory allocation failed", false, 0);
        vTaskDelete(NULL);
        return;
      }
      // Inherit the parent thread's priority
      UBaseType_t parent_priority = uxTaskPriorityGet(NULL);
      xTaskCreate(
        can_rtr_message_handler_taks, // Task function
        "can_rtr_handler",            // Name of the task
        2048,                        // Stack size in words
        rtr_message_ptr,             // Task parameters (pointer to received message)
        parent_priority,             // Task priority
        NULL                         // Task handle
      );

    } else {
      debug_msg(DEBUG_PARTIAL_CAN, "Received a data message for distribution controller", false, 0);

      // determine broadcast message type
      switch (received_message.data[0]) {
        case can_paket_type_io_state:
          debug_msg(DEBUG_PARTIAL_CAN, "Received IO state message", false, 0);

          // do nothing not applicable for distribution controller

          break;
        case can_paket_type_status:
          debug_msg(DEBUG_PARTIAL_CAN, "Received status message", false, 0);

          // check if allow_off_road is enabled
          if ((received_message.data[1] & 0b00100000) != 0) {
            debug_msg(DEBUG_PARTIAL_CAN, "Allow off-road is enabled", false, 0);
            io_allow_off_road = true;
          } else {
            debug_msg(DEBUG_PARTIAL_CAN, "Allow off-road is disabled", false, 0);
            io_allow_off_road = false;
          }

          // check if blackout is enabled
          if ((received_message.data[1] & 0b00010000) != 0) {
            debug_msg(DEBUG_PARTIAL_CAN, "Blackout is enabled", false, 0);
            io_blackout_enabled = true;
          } else {
            debug_msg(DEBUG_PARTIAL_CAN, "Blackout is disabled", false, 0);
            io_blackout_enabled = false;
          }

          break;
        case can_paket_type_chanel_set_curent_limit:
          debug_msg(DEBUG_PARTIAL_CAN, "Received channel set current limit message", false, 0);

          // find pin that needs to be set
          for (size_t i = 1; i < sizeof(pin_names) / sizeof(pin_names[0]); i++) {
            if (pin_names[i]->pin_mode == analog_in) {
              if (pin_names[i]->io_chanel_number == received_message.data[1]) {
                // Capture the current pin pointer and current_limit in a struct to pass to the task
                struct AdcTaskParams {
                  pin* p;
                  uint current_limit;
                };
                AdcTaskParams* params = new AdcTaskParams{
                  pin_names[i],
                  (uint)((received_message.data[2] << 8) | received_message.data[3])
                };
                // Inherit the parent thread's priority
                UBaseType_t parent_priority = uxTaskPriorityGet(NULL);
                xTaskCreate([](void* pvParameters) {
                    AdcTaskParams* taskParams = static_cast<AdcTaskParams*>(pvParameters);
                    adc_set_ch_current_limit_rtos_wraper(taskParams);
                    delete taskParams; // Free the allocated memory
                    vTaskDelete(NULL); // Delete the task
                }, "adc_set_chanel_current_limit", 2048, (void*)params, parent_priority, NULL);
                break; // Exit the loop once the pin is found and task is created
              }
            }
          }

          break;
        case can_paket_type_chanel_control:
          debug_msg(DEBUG_PARTIAL_CAN, "Received channel control message", false, 0);

          // find pin that needs to be controlled
          for (size_t i = 1; i < sizeof(pin_names[0]); i++) {
            if (pin_names[i]->pin_mode == out) {
              if (pin_names[i]->io_chanel_number == received_message.data[6]) {

                // Prepare parameters for the RTOS task
                struct IoTaskParams {
                  pin* p;
                  high_low state;
                };

                high_low state;
                if ((received_message.data[2] & 0x80) == 0) {
                  debug_msg(DEBUG_PARTIAL_CAN, "Channel control message: setting pin to LOW", false, 0);
                  state = low; // Set state to low if the 7th bit is 0
                } else {
                  debug_msg(DEBUG_PARTIAL_CAN, "Channel control message: setting pin to HIGH", false, 0);
                  state = high;
                }

                // Use a lambda to ensure memory is always freed, even if io_call_rtos_wraper throws
                IoTaskParams* params = new IoTaskParams{
                  pin_names[i],
                  state
                };
                UBaseType_t parent_priority = uxTaskPriorityGet(NULL);
                xTaskCreate([](void* pvParameters) {
                    IoTaskParams* taskParams = static_cast<IoTaskParams*>(pvParameters);
                    if (taskParams) {
                      io_call_rtos_wraper(taskParams);
                      delete taskParams; // Free the allocated memory
                    }
                    vTaskDelete(NULL); // Delete the task
                }, "io_channel_control", 2048, (void*)params, parent_priority, NULL);
                break; // Exit the loop once the pin is found
              }
            }
          }
          break;
        default:
          break;
      }
    }
  }
}


void can_rtr_message_handler_taks (void *pvParameters) {
  // Cast the parameter to a pointer to struct message
  struct message* received_message_ptr = (struct message*)pvParameters;
  if (received_message_ptr == nullptr) {
    debug_msg(DEBUG_PARTIAL_CAN, "can_rtr_message_handler_taks: null parameter", false, 0);
    vTaskDelete(NULL);
    return;
  }
  struct message received_message = *received_message_ptr;
  // Free the parameter memory if it was dynamically allocated
  delete received_message_ptr;

  debug_msg(DEBUG_PARTIAL_CAN, "can_rtr_message_handler_taks called", false, 0);

  switch (received_message.data[0]) {
    case can_paket_type_io_state:
      debug_msg(DEBUG_PARTIAL_CAN, "RTR for IO state received", false, 0);
      
      //create state vars
      uint32_t io_state = 0;

      //struct for return message
      struct message io_state_message;
      io_state_message.id = received_message.id;
      io_state_message.extended = received_message.extended;
      io_state_message.type = FRAME_DATA; // Set the frame type to data
      io_state_message.single_shot = false; // Set single shot flag to false
      io_state_message.data[0] = can_paket_type_io_state; // Set the first byte to the IO state packet type

      // read the curent chanel states
      xTaskCreate(
        io_read_chanel_state, // Task function
        "io_read_rtos",      // Name of the task
        2048,                // Stack size in words
        (void*)channelStateQueue, // Task parameters (pointer to received message)
        uxTaskPriorityGet(NULL), // Inherit parent thread's priority
        NULL                  // Task handle
      );

      if (xQueueReceive(channelStateQueue, &io_state, pdMS_TO_TICKS(1000)) == pdTRUE) {
        debug_msg(DEBUG_PARTIAL_CAN, "IO state read successfully", false, 0);
      } else {
        debug_msg(DEBUG_PARTIAL_CAN, "Failed to read IO state", false, 0);
      }

      can_send_frame(io_state_message); // Send the IO state message
      debug_msg(DEBUG_PARTIAL_CAN, "IO state message sent", false, 0);
      // Clean up the queue after use
      xQueueReset(channelStateQueue); // Reset the queue for future use

      break;
    case can_paket_type_adc_read:
      debug_msg(DEBUG_PARTIAL_CAN, "RTR for ADC read received", false, 0);
      // create vars
      uint16_t vref_reading = 0;
      uint16_t channel_reading = 0;
      struct pin pin_to_read;

      //struct for return message
      struct message adc_reading_message;
      adc_reading_message.id = received_message.id;
      adc_reading_message.extended = received_message.extended;
      adc_reading_message.type = FRAME_DATA; // Set the frame type to data
      adc_reading_message.single_shot = false; // Set single shot flag to false

      //find what channel to read
      for (size_t i = 1; i < sizeof(pin_names) / sizeof(pin_names[0]); i++) {
        if (pin_names[i]->adc_channel == received_message.data[1]) {
          pin_to_read = *pin_names[i]; // Copy the pin structure
          debug_msg(DEBUG_PARTIAL_CAN, "Found pin to read ADC:", true, pin_to_read.pin_number);
          break;
        }
      }

      // create task to read adc, passing pin_to_read as parameter
      xTaskCreate(
        adc_read_rtos_wraper, // Task function
        "adc_read_rtos",      // Name of the task
        2048,                 // Stack size in words
        (void*)&pin_to_read,  // Pass pointer to pin_to_read
        uxTaskPriorityGet(NULL), // Inherit parent thread's priority
        NULL                  // Task handle
      );

      //read adc value from queue
      if (xQueueReceive(adc_reading_queue, &channel_reading, pdMS_TO_TICKS(1000)) == pdTRUE) {
        debug_msg(DEBUG_PARTIAL_CAN, "ADC channel reading received", true, channel_reading);
      } else {
        debug_msg(DEBUG_PARTIAL_CAN, "Failed to read ADC channel", false, 0);
      }
      break;

      vref_reading = analogRead(ref_12v.pin_number); // Read the Vref pin

      //transmit response
      adc_reading_message.data[0] = can_paket_type_adc_read; // Set the first byte to the ADC read packet type
      adc_reading_message.data[1] = pin_to_read.adc_channel; // Set the ADC channel number
      adc_reading_message.data[2] = channel_reading & 0xFF; // Set the high byte of Vref reading
      adc_reading_message.data[3] = (channel_reading >> 8) & 0xFF; // Set the low byte of Vref reading
      adc_reading_message.data[4] = vref_reading & 0xFF; // Set the high byte of Vref reading
      adc_reading_message.data[5] = (vref_reading >> 8) & 0xFF; // Set the low byte of Vref reading

      can_send_frame(adc_reading_message); // Send the ADC reading message
      debug_msg(DEBUG_PARTIAL_CAN, "ADC reading message sent", false, 0);
      // Clean up the queue after use
      xQueueReset(adc_reading_queue); // Reset the queue for future use
      break;
      
    case can_paket_type_status:
      debug_msg(DEBUG_PARTIAL_CAN, "RTR for status received", false, 0);
      // Handle RTR for status
      // This could involve sending the current status of the system back to the requester
      break;
    case can_paket_type_chanel_set_curent_limit:
      debug_msg(DEBUG_PARTIAL_CAN, "RTR for channel set current limit received", false, 0);
      // Handle RTR for channel set current limit
      // This could involve sending the current limit of a specific channel back to the requester
      break;
    case can_paket_type_total_current:
      debug_msg(DEBUG_PARTIAL_CAN, "RTR for total current received", false, 0);
      // Handle RTR for total current
      // This could involve sending the total current consumption back to the requester
      break;
    default:
      debug_msg(DEBUG_PARTIAL_CAN, "Unknown RTR type received", false, 0);
      break;
  }

  vTaskDelete(NULL); // Delete the task after processing
}







#endif // can_cpp