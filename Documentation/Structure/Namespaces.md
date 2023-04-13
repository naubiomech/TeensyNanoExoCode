# Namespaces

## Exocode.ino

### config_info 
- config_to_send

## Board.h
### logic_micro_pins
- rx1_pin
- tx1_pin
- can_rx_pin
- can_tx_pin
- fsr_sense_left_heel_pin
- fsr_sense_left_toe_pin
- fsr_sense_right_heel_pin
- fsr_sense_right_toe_pin
- num_available_joints
- torque_sensor_left[]
- torque_sensor_right[]
- sync_led_pin
- sync_default_pin
- sync_led_on_state
- sync_led_off_state
- status_led_r_pin
- status_led_g_pin
- status_led_b_pin
- status_has_pwm
- status_led_on_state
- miso_pin
- mosi_pin
- sck_pin
- cs_pin
- irq_pin
- rst_pin
- motor_stop_pin
- not_connected_pin
- enable_left_pin[]
- enable_right_pin[]
- motor_enable_on_state
- motor_enable_off_state 

### coms_micro_pins
- ble_signal_pin
- ble_signal_active
- miso_pin
- mosi_pin
- sck_pin
- cs_pin
- spi_mode 
    
## Config.h

### sync_time 
- NUM_START_STOP_BLINKS
- SYNC_HALF_PERIOD_US
- SYNC_START_STOP_HALF_PERIOD_US 
    
### torque_calibration
- AI_CNT_TO_V
- TRQ_V_TO_NM
 
### BLE_times
- _status_msg_delay
- _real_time_msg_delay

## ParseIni.h

### ini_config
- buffer_length
- key_length
- section_length
- number_of_keys

### config_defs
- board_name
- board_version
- battery
- exo_name
- exo_side
- JointType
- motor
- gearing
- joint_id
- hip_controllers
- knee_controllers
- ankle_controllers
- flip_dir
- board_name_idx
- board_version_idx
- battery_idx
- exo_name_idx
- exo_side_idx
- hip_idx
- knee_idx
- ankle_idx
- hip_gear_idx
- knee_gear_idx
- ankle_gear_idx
- exo_hip_default_controller_idx
- exo_knee_default_controller_idx
- exo_ankle_default_controller_idx
- hip_flip_dir_idx
- knee_flip_dir_idx
- ankle_flip_dir_idx

### config_map
- board_name
- board_version
- battery
- exo_name
- exo_side
- motor
- gearing
- hip_controllers
- knee_controllers
- ankle_controllers
- flip_dir
    
## ble_commands.h
### names
- start
- stop
- cal_trq
- cal_fsr
- new_trq
- new_fsr     
- assist
- resist
- motors_on     
- motors_off
- mark
- send_real_time_data     
- send_batt
- send_cal_done
- send_error_count     
- send_trq_cal
- send_step_count
- cal_fsr_finished

### ble
- commands

### ble_command_helpers
- int get_length_for_command(char command)

### handlers
- start(ExoData* data, BleMessage* msg)
- stop(ExoData* data, BleMessage* msg)
- cal_trq(ExoData* data, BleMessage* msg)
- cal_fsr(ExoData* data, BleMessage* msg)
- assist(ExoData* data, BleMessage* msg)
- resist(ExoData* data, BleMessage* msg)
- motors_on(ExoData* data, BleMessage* msg)
- motors_off(ExoData* data, BleMessage* msg)
- mark(ExoData* data, BleMessage* msg)
- new_trq(ExoData* data, BleMessage* msg)
- new_fsr(ExoData* data, BleMessage* msg)

## ExoBLE.h

### ble_rx
- on_rx_recieved(BLEDevice central, BLECharacteristic characteristic);
- on_rx_recieved(char data[], uint16_t len);

## I2CHandler.h

### i2c_cmds
- get_battery_voltage
    - addr
    - reg
    - len
- get_battery_soc
    - addr
    - reg
    - len
 
## ParamsFromSD.h
### param_error
- num_joint_ids
- SD_not_found_idx
- file_not_found_idx

### controller_parameter_filenames
- hip
- knee
- ankle

## StatusDefs.h
### status_defs
- messages
    - off
    - trial_off
    - trial_on
    - test
    - torque_calibration
    - fsr_calibration
    - fsr_refinement
    - error_bit
    - error
    - error_left_heel_fsr
    - error_left_toe
    - error_right_heel_fsr
    - error_right_toe_fsr
    - error_left_hip_torque_sensor
    - error_left_knee_torque_sensor
    - error_left_ankle_torque_sensor
    - error_right_hip_torque_sensor
    - error_right_knee_torque_sensor
    - error_right_ankle_torque_sensor
    - error_left_hip_motor
    - error_left_knee_motor
    - error_left_ankle_motor
    - error_right_hip_motor
    - error_right_knee_motor
    - error_right_ankle_motor
    - error_left_hip_controller
    - error_left_knee_controller
    - error_left_ankle_controller
    - error_right_hip_controller
    - error_right_knee_controller
    - error_right_ankle_controller 
    
## StatusLed.h
- status_led_defs   
    - status_led_idx 
    - colors
        - off[]
        - trial_off[]
        - trial_on[]
        - test[]
        - error[]
        - torque_calibration[]
        - fsr_calibration[]
        - fsr_refinement[] 
    - patterns
        - solid
        - blink
        - pulse
        - rainbow
        - off[]
        - trial_off[]
        - trial_on[]
        - test[]
        - error[]
        - torque_calibration[]
        - fsr_calibration[]
        - fsr_refinement[]
    - on_state
    - off_state
    - has_pwm
    
## Utilities.h   
### utils
- bool get_is_left(config_defs::joint_id id)
- bool get_is_left(uint8_t id)
- uint8_t get_joint_type(config_defs::joint_id id)
- uint8_t get_joint_type(uint8_t id)
- bool schmitt_trigger(float value, bool is_high, float lower_threshold, float upper_threshold)
- int rate_limit(int setpoint, int last_value, int* last_time, int rate_per_ms)
- uint8_t update_bit(uint8_t original, bool val, uint8_t loc)
- uint16_t update_bit(uint16_t original, bool val, uint8_t loc)    
- bool get_bit(uint8_t original, uint8_t loc)
- bool get_bit(uint16_t original, uint8_t loc)
- float degrees_to_radians(float)
- float radians_to_degrees(float)
- String remove_all_chars(String str, char rmv)
- String remove_all_chars(char* arr, int len, char rmv)
- int get_char_length(int ofInt)
- int elements_are_equal(T arr1, T arr2, int length)
- set_elements_equal(T arr1, T arr2, int length)
- class SpeedCheck
- bool is_little_endian()
- float_to_uint8(float num_to_convert, uint8_t *converted_bytes)
- uint8_to_float(uint8_t *bytes_to_convert, float *converted_float)