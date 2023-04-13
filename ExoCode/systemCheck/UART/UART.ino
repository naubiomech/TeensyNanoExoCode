#include "src\Utilities.h"
#include "src\UARTHandler.h"
#include "src\UART_commands.h"
#include "src\UART_msg_t.h"
#include "src\ParseIni.h"

#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
static const float k_timeout_us = 2500;
namespace config_info
{
    uint8_t (config_to_send)[ini_config::number_of_keys];
}

#elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
static const float k_timeout_us = 1000;
namespace config_info
{
    uint8_t config_to_send[ini_config::number_of_keys] = {
        1,  // board name
        3,  // board version
        2,  // battery
        1,  // exo name
        1,  // exo side
        2,  // hip
        1,  // knee
        3,  // ankle
        1,  // hip gear
        1,  // knee gear
        1,  // ankle gear
        1,  // hip default controller
        1,  // knee default controller
        1,  // ankle default controller
        3,  // hip flip dir
        3,  // knee flip dir
        3,  // ankle flip dir
      };
}
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial);  
}

void loop() {
  logger::println("===============================================================================================");
  
  UARTHandler* inst = UARTHandler::get_instance();

  static uint8_t first_run{1};
  if (first_run) {
    first_run = 0;
    /* Nano needs the config data */
    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
    logger::println("Loop->Getting config");
    UART_command_utils::get_config(inst, config_info::config_to_send);
    logger::println("Loop->Got config: ");
    for (int i=0; i<ini_config::number_of_keys; i++)
    {
      logger::print(config_info::config_to_send[i]); logger::print(", ");
    }
    logger::println(k_timeout_us);
    
    #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    UART_command_utils::wait_for_get_config(inst, config_info::config_to_send);
    #endif
  }
  
  static ExoData exo_data(config_info::config_to_send);
  
  /* How to check for, receive, and handle a UART command */
  UART_msg_t msg = inst->poll(k_timeout_us);
  if (msg.command) {
    UART_command_utils::handle_msg(inst, &exo_data, msg);
  }
  
  static float old_time = micros();
  float now = micros();
  float delta = now-old_time;
  if (delta > 20)
  {
    /* How to send a UART command */
    #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
    static float p_gain = 300;
    // pack the message that you would like to send
    UART_msg_t tx_msg;
    tx_msg.command = UART_command_names::update_controller_params;
    tx_msg.joint_id = (uint8_t)config_defs::joint_id::left_hip;
    tx_msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID] = (float)config_defs::hip_controllers::zero_torque;
    tx_msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_LENGTH] = (float)controller_defs::zero_torque::num_parameter;
    tx_msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + (uint8_t)controller_defs::zero_torque::use_pid_idx] = 1;
    tx_msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + (uint8_t)controller_defs::zero_torque::p_gain_idx] = p_gain++;
    tx_msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + (uint8_t)controller_defs::zero_torque::i_gain_idx] = 0;
    tx_msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + (uint8_t)controller_defs::zero_torque::d_gain_idx] = 6;
    tx_msg.len = (uint8_t)UART_command_enums::controller_params::PARAM_START + (uint8_t)controller_defs::zero_torque::num_parameter;
    // Send the message
    inst->UART_msg(tx_msg);
  
    logger::println("Loop->Updated controller params");
    #elif defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    UART_msg_t tx_msg;
    tx_msg.command = UART_command_names::update_status;
    tx_msg.joint_id = 0;
    tx_msg.data[(uint8_t)UART_command_enums::status::STATUS] = exo_data.status++;
    tx_msg.len = (uint8_t)UART_command_enums::status::LENGTH;
    inst->UART_msg(tx_msg);
    logger::println("Loop->Updated status");
    #endif
    logger::println("===============================================================================================");
    old_time = now;
  }
}
