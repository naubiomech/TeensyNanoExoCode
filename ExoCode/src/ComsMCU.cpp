#include "ComsMCU.h"
#include "StatusLed.h"
#include "StatusDefs.h"
#include "Time_Helper.h"
#include "UARTHandler.h"
#include "uart_commands.h"
#include "UART_msg_t.h"
#include "Config.h"
#include "error_types.h"
#include "Logger.h"

#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

#define COMSMCU_DEBUG 1

ComsMCU::ComsMCU(ExoData* data, uint8_t* config_to_send):_data{data}
{
    switch (config_to_send[config_defs::battery_idx])
    {
    case (uint8_t)config_defs::battery::smart:
        _battery = new SmartBattery();
        break;
    case (uint8_t)config_defs::battery::dumb:
        _battery = new RCBattery();
        break;
    default:
        //logger::println("ERROR: ComsMCU::ComsMCU->Unrecognized battery type!");
        _battery = new RCBattery();
        break;
    }
    _battery->init();
    _exo_ble = new ExoBLE();
    _exo_ble->setup();


    uint8_t rt_data_len = 0;
    switch (config_to_send[config_defs::exo_name_idx])
    {
        case (uint8_t)config_defs::exo_name::bilateral_ankle:
            rt_data_len = rt_data::BILATERAL_ANKLE_RT_LEN;
            break;
        case (uint8_t)config_defs::exo_name::bilateral_hip:
            rt_data_len = rt_data::BILATERAL_HIP_RT_LEN;
            break;
        case (uint8_t)config_defs::exo_name::bilateral_hip_ankle:
            rt_data_len = rt_data::BILATERAL_HIP_ANKLE_RT_LEN;
            break;
        default:
            rt_data_len = 8;
            break;
    }
    //rt_data::msg_len = rt_data_len
    // logger::print("ComsMCU::ComsMCU->rt_data_len: "); logger::println(rt_data_len);
}

void ComsMCU::handle_ble()
{
    bool non_empty_ble_queue = _exo_ble->handle_updates();
    if (non_empty_ble_queue)
    {
        BleMessage msg = ble_queue::pop();
        _process_complete_gui_command(&msg);
    }
}

void ComsMCU::local_sample()
{
    Time_Helper* t_helper = Time_Helper::get_instance();
    static const float context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(context);
    if (del_t > (BLE_times::_status_msg_delay/2)) 
    {
        static float filtered_value = _battery->get_parameter();
        float raw_battery_value = _battery->get_parameter();
        filtered_value = utils::ewma(raw_battery_value, filtered_value, k_battery_ewma_alpha);
        _data->battery_value = filtered_value;
        del_t = 0;
    }
}

void ComsMCU::update_UART()
{
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static const float _context = t_helper->generate_new_context();
    static float del_t = 0;
    del_t += t_helper->tick(_context);
    
    if (del_t > UART_times::UPDATE_PERIOD)
    {
        UARTHandler* handler = UARTHandler::get_instance();
        UART_msg_t msg = handler->poll(UART_times::COMS_MCU_TIMEOUT);
        if (msg.command) 
        {
            // UART_msg_t_utils::print_msg(msg);
            UART_command_utils::handle_msg(handler, _data, msg);
        }
        del_t = 0;
    }
}


void ComsMCU::update_gui() 
{
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static float my_mark = _data->mark;
    static float* rt_floats = new float(rt_data::len);

    // Get real time data from ExoData and send to GUI
    const bool new_rt_data = real_time_i2c::poll(rt_floats);
    if (new_rt_data || rt_data::new_rt_msg)
    {
        _life_pulse();
        rt_data::new_rt_msg = false;
        // float now = millis();
        // float delta = now - before;
        // logger::print("ComsMCU::update_gui->delta: "); logger::println(delta);
        // before = now;

        BleMessage rt_data_msg = BleMessage();
        rt_data_msg.command = ble_names::send_real_time_data;
        rt_data_msg.expecting = rt_data::len;

        for (int i = 0; i < rt_data::len; i++)
        {   
            #if REAL_TIME_I2C
            rt_data_msg.data[i] = rt_floats[i];
            #else
            rt_data_msg.data[i] = rt_data::float_values[i];
            #endif
        }

        //rt_data_msg.data[rt_data_msg.expecting++] = 0;//time_since_last_message/1000.0;

        if (my_mark < _data->mark)
        {
            my_mark = _data->mark;
            rt_data_msg.data[_mark_index] = my_mark;
        }

        _exo_ble->send_message(rt_data_msg);
    }

    // Periodically send status information
    static float status_context = t_helper->generate_new_context(); //Took out "const" after static and before float - Jack 
    static float del_t_status = 0;
    del_t_status += t_helper->tick(status_context);
    if (del_t_status > BLE_times::_status_msg_delay)
    {
        // Send status data
        BleMessage batt_msg = BleMessage();
        batt_msg.command = ble_names::send_batt;
        batt_msg.expecting = ble_command_helpers::get_length_for_command(batt_msg.command);
        batt_msg.data[0] = _data->battery_value;
        _exo_ble->send_message(batt_msg);

        del_t_status = 0;
    }
}

void ComsMCU::handle_errors()
{
    static int error_code = NO_ERROR;
    if (_data->error_code != error_code)
    {
        error_code = _data->error_code;
        _exo_ble->send_error(_data->error_code, _data->error_joint_id);
    }
}

void ComsMCU::_process_complete_gui_command(BleMessage* msg) 
{
    // logger::print("ComsMCU::_process_complete_gui_command->Got Command: ");
    // BleMessage::print(*msg);

    switch (msg->command)
    {
    case ble_names::start:
        ble_handlers::start(_data, msg);
        break;
    case ble_names::stop:
        ble_handlers::stop(_data, msg);
        break;
    case ble_names::cal_trq:
        ble_handlers::cal_trq(_data, msg);
        break;
    case ble_names::cal_fsr:
        ble_handlers::cal_fsr(_data, msg);
        break;
    case ble_names::assist:
        ble_handlers::assist(_data, msg);
        break;
    case ble_names::resist:
        ble_handlers::resist(_data, msg);
        break;
    case ble_names::motors_on:
        ble_handlers::motors_on(_data, msg);
        break;
    case ble_names::motors_off:
        ble_handlers::motors_off(_data, msg);
        break;
    case ble_names::mark:
        ble_handlers::mark(_data, msg);
        break;
    case ble_names::new_fsr:
        ble_handlers::new_fsr(_data, msg);
        break;
    case ble_names::new_trq:
        ble_handlers::new_trq(_data, msg);
        break;
    case ble_names::perturb:
        ble_handlers::perturb(_data, msg);
        break;
    case ble_names::update_param:
        ble_handlers::update_param(_data, msg);
        break;
    default:
        logger::println("ComsMCU::_process_complete_gui_command->No case for command!", LogLevel::Error);
        break;
    }
}


void ComsMCU::_life_pulse()
{
    static int count = 0;
    count++;
    if (count > k_pulse_count)
    {
        count = 0;
        digitalWrite(25, !digitalRead(25));
    }
}
#endif