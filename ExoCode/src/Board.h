/**
 * @file Board.h
 * @author Chancelor Cuddeback
 * @brief Defines the pins and peripherals for the board being used.
 * @date 2023-07-18
 * 
 */

#ifndef BOARD_SETTINGS_HEADER
    #define BOARD_SETTINGS_HEADER
    // include config so we know what board is being used.
    #include "Config.h"

    #if BOARD_VERSION == AK_Board_V0_1
        #include "Arduino.h"
        namespace logic_micro_pins  //teensy
        {
            #if defined(ARDUINO_TEENSY36)
                // Serial Pins, NC
                const unsigned int rx1_pin = 0;
                const unsigned int tx1_pin = 1;
                
                // CAN Pins
                const unsigned int can_rx_pin = 4;
                const unsigned int can_tx_pin = 3;
                
                // FSR Pins
                const unsigned int fsr_sense_left_heel_pin = A14;
                const unsigned int fsr_sense_left_toe_pin = A15;
                const unsigned int fsr_sense_right_heel_pin= A7;
                const unsigned int fsr_sense_right_toe_pin = A6;
                
                // Torque Sensor Pins
                const unsigned int num_available_joints = 2;
                const unsigned int torque_sensor_left[] = {A17, A16};
                //const unsigned int torque_sensor_left1 = A16;
                const unsigned int torque_sensor_right[] = {A9, A8};
                //const unsigned int torque_sensor_right1 = A8;
                
                
                // Sync LED Pins
                const unsigned int sync_led_pin = 29;
                const unsigned int sync_default_pin = 25;
            #endif
            const unsigned int sync_led_on_state = LOW;//HIGH;
            const unsigned int sync_led_off_state = HIGH;//LOW;
            
            // Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
            #if defined(ARDUINO_TEENSY36)
                // Status LED Pins
                const unsigned int status_led_r_pin= 28;
                const unsigned int status_led_g_pin = 27;
                const unsigned int status_led_b_pin = 26;
            #endif
            // if you have connected to pins with PWM set to true.
            const bool status_has_pwm = false;
            // For high to be on use 255 for the on state and 0 for the off, for low as on flip it.
            const uint8_t status_led_on_state = 0;//255;
            const uint8_t status_led_off_state = 4095;//0;
            
            // Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
            #if defined(ARDUINO_TEENSY36)
                // SPI Follower Pins
                const unsigned int miso_pin = 12;
                const unsigned int mosi_pin= 11;
                const unsigned int sck_pin = 13;
                const unsigned int cs_pin = 10;
                const unsigned int spi_mode = 16;
                
                // Pin to Stop the Motors
                const unsigned int motor_stop_pin = 6;
            #endif
            // Pin to use when we need a value but don't actually want to use it.
            const unsigned int not_connected_pin = 42;  // selected 42 as it is a pad on the back so I figure it won't hurt anything if something goes wrong.
            
            const unsigned int enable_left_pin[] = {not_connected_pin, not_connected_pin};
            const unsigned int enable_right_pin[] = {not_connected_pin, not_connected_pin};
            
            const unsigned int motor_enable_on_state = HIGH;
            const unsigned int motor_enable_off_state = LOW;
        };
        
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
            namespace coms_micro_pins  //nano
            {
                const unsigned int blue = 24;
                const unsigned int green = 23;
                const unsigned int red = 22;
                const unsigned int led_active_low = 1;
            };
        #endif    
    #elif BOARD_VERSION == AK_Board_V0_3 
       
        #include "Arduino.h"
        namespace logic_micro_pins  //teensy
        {
             #if defined(ARDUINO_TEENSY41)
                // Serial Pins, NC
                const unsigned int rx1_pin = 0;
                const unsigned int tx1_pin = 1;
                
                // CAN Pins
                const unsigned int can_rx_pin = 23;
                const unsigned int can_tx_pin = 22;
                
                // FSR Pins
                const unsigned int fsr_sense_left_heel_pin = A14;
                const unsigned int fsr_sense_left_toe_pin = A15;
                const unsigned int fsr_sense_right_heel_pin= A5;
                const unsigned int fsr_sense_right_toe_pin = A4;
                
                // Torque Sensor Pins
                const unsigned int num_available_joints = 2;
                const unsigned int torque_sensor_left[] = {A12, A13};
                //const unsigned int torque_sensor_left1 = A16;
                const unsigned int torque_sensor_right[] = {A7, A6};
                //const unsigned int torque_sensor_right1 = A8;
                
                
                // Sync LED Pins
                const unsigned int sync_led_pin = 15;
                const unsigned int sync_default_pin = 5;
            #endif
            // Arduino compiles all files not just the ones that are used so this is not under teensy to prevent errors
            const unsigned int sync_led_on_state = LOW;//HIGH;
            const unsigned int sync_led_off_state = HIGH;//LOW;

             #if defined(ARDUINO_TEENSY41)
                // Status LED Pins
                const unsigned int status_led_r_pin = 14;
                const unsigned int status_led_g_pin = 25;
                const unsigned int status_led_b_pin = 24;
            #endif

            // if you have connected to pins with PWM set to true.
            const bool status_has_pwm = true;             
            // For high to be on use 255 for the on state and 0 for the off, for low as on flip it.
            const uint8_t status_led_on_state = 0;//255;
            const uint8_t status_led_off_state = 4095;//0;  
                
            #if defined(ARDUINO_TEENSY41)    
                // SPI Follower Pins
                const unsigned int miso_pin = 12;
                const unsigned int mosi_pin= 11;
                const unsigned int sck_pin = 13;
                const unsigned int cs_pin = 10;
                const unsigned int irq_pin = 34;
                const unsigned int rst_pin = 4;
                const unsigned int spi_mode = 8; // This is 8 or 16 bit, not the actual SPI mode, I know it is confusing but that is how they chose to make the library.
                
                
                // Pin to Stop the Motors
                const unsigned int motor_stop_pin = 9;
                
                // Pin to use when we need a value but don't actually want to use it.
                const unsigned int not_connected_pin = 51;  // selected 51 as it is a pad on the back so I figure it won't hurt anything if something goes wrong.
                
                // Motor enable Pins
                const unsigned int enable_left_pin[] = {28, 29};
                const unsigned int enable_right_pin[] = {8, 7};
                
                
                const unsigned int speed_check_pin = 33;

                const unsigned int left_ankle_angle_pin = A16;
                const unsigned int right_ankle_angle_pin = A17;
                
                // I2C 
                // SDA 18
                // SCL 19
                
                //SPI
                // 
            #endif
            
            const unsigned int motor_enable_on_state = HIGH;
            const unsigned int motor_enable_off_state = LOW;
        };
        
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        namespace coms_micro_pins  //nano
        {
            const unsigned int blue = 24;
            const unsigned int green = 23;
            const unsigned int red = 22;
            const unsigned int led_active_low = 1;
            
            // SPI Conroller Pins
            const unsigned int miso_pin = 11;
            const unsigned int mosi_pin= 12;
            const unsigned int sck_pin = 13;
            const unsigned int cs_pin = 10;
            const unsigned int spi_mode = 16;
            
            
        };
        #endif
    #elif BOARD_VERSION == AK_Board_V0_4 
       
        #include "Arduino.h"
        namespace logic_micro_pins  //teensy
        {
             #if defined(ARDUINO_TEENSY41)
                // Serial Pins, NC
                const unsigned int rx1_pin = 0;
                const unsigned int tx1_pin = 1;
                
                // CAN Pins
                const unsigned int can_rx_pin = 23;
                const unsigned int can_tx_pin = 22;
                
                // FSR Pins
                const unsigned int fsr_sense_left_heel_pin = A14;
                const unsigned int fsr_sense_left_toe_pin = A15;
                const unsigned int fsr_sense_right_heel_pin= A3;
                const unsigned int fsr_sense_right_toe_pin = A2;
                
                // Torque Sensor Pins
                const unsigned int num_available_joints = 2;
                const unsigned int torque_sensor_left[] = {A17, A16};
                //const unsigned int torque_sensor_left1 = A16;
                const unsigned int torque_sensor_right[] = {A7, A6};
                //const unsigned int torque_sensor_right1 = A8;
                
                
                // Sync LED Pins
                const unsigned int sync_led_pin = 15;
                const unsigned int sync_default_pin = 5;
            #endif
            // Arduino compiles all files not just the ones that are used so this is not under teensy to prevent errors
            const unsigned int sync_led_on_state = LOW;//HIGH;
            const unsigned int sync_led_off_state = HIGH;//LOW;

             #if defined(ARDUINO_TEENSY41)
                // Status LED Pins
                const unsigned int status_led_r_pin= 14;
                const unsigned int status_led_g_pin = 25;
                const unsigned int status_led_b_pin = 24;
            #endif

            // if you have connected to pins with PWM set to true.
            const bool status_has_pwm = true;             
            // For high to be on use 255 for the on state and 0 for the off, for low as on flip it.
            const uint8_t status_led_on_state = 0;//255;
            const uint8_t status_led_off_state = 4095;//0;  
                
            #if defined(ARDUINO_TEENSY41)    
                // SPI Follower Pins
                const unsigned int miso_pin = 12;
                const unsigned int mosi_pin= 11;
                const unsigned int sck_pin = 13;
                const unsigned int cs_pin = 10;
                const unsigned int irq_pin = 34;
                const unsigned int rst_pin = 4;
                const unsigned int spi_mode = 8; // This is 8 or 16 bit, not the actual SPI mode, I know it is confusing but that is how they chose to make the library.
                
                
                // Pin to Stop the Motors
                const unsigned int motor_stop_pin = 9;
                
                // Pin to use when we need a value but don't actually want to use it.
                const unsigned int not_connected_pin = 51;  // selected 51 as it is a pad on the back so I figure it won't hurt anything if something goes wrong.
                
                // Motor enable Pins
                const unsigned int enable_left_pin[] = {28, 29};
                const unsigned int enable_right_pin[] = {8, 7};
                
                
                const unsigned int speed_check_pin = 33;

                const unsigned int left_ankle_angle_pin;
                const unsigned int right_ankle_angle_pin;
                
                // I2C 
                // SDA 18
                // SCL 19
                
                //SPI
                // 
            #endif
            
            const unsigned int motor_enable_on_state = HIGH;
            const unsigned int motor_enable_off_state = LOW;
        };
        
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        namespace coms_micro_pins  //nano
        {
            const unsigned int blue = 24;
            const unsigned int green = 23;
            const unsigned int red = 22;
            const unsigned int led_active_low = 1;
            
            // SPI Conroller Pins
            const unsigned int miso_pin = 11;
            const unsigned int mosi_pin= 12;
            const unsigned int sck_pin = 13;
            const unsigned int cs_pin = 10;
            const unsigned int spi_mode = 16;
            
            
        };
        #endif
    #elif BOARD_VERSION == AK_Board_V0_5_1 
       
        #include "Arduino.h"
        namespace logic_micro_pins  //teensy
        {
             #if defined(ARDUINO_TEENSY41)
                // Serial Pins, NC
                const unsigned int rx1_pin = 0;
                const unsigned int tx1_pin = 1;
                
                // CAN Pins
                const unsigned int can_rx_pin = 23;
                const unsigned int can_tx_pin = 22;
                
                // FSR Pins
                const unsigned int fsr_sense_left_heel_pin = A14;
                const unsigned int fsr_sense_left_toe_pin = A15;
                const unsigned int fsr_sense_right_heel_pin= A3;
                const unsigned int fsr_sense_right_toe_pin = A2;
                
                // Torque Sensor Pins
                const unsigned int num_available_joints = 2;
                //const unsigned int torque_sensor_left[] = {A16, A17};
				const unsigned int torque_sensor_left[] = {A16};
                //const unsigned int torque_sensor_left1 = A16;
                //const unsigned int torque_sensor_right[] = {A6, A7};
				const unsigned int torque_sensor_right[] = {A6};
                //const unsigned int torque_sensor_right1 = A8;
                
                
                // Sync LED Pins
                const unsigned int sync_led_pin = 15;
                const unsigned int sync_default_pin = 5;
            #endif
            // Arduino compiles all files not just the ones that are used so this is not under teensy to prevent errors
            const unsigned int sync_led_on_state = LOW;//HIGH;
            const unsigned int sync_led_off_state = HIGH;//LOW;

             #if defined(ARDUINO_TEENSY41)
                // Status LED Pins
                const unsigned int status_led_r_pin= 14;
                const unsigned int status_led_g_pin = 25;
                const unsigned int status_led_b_pin = 24;
            #endif

            // if you have connected to pins with PWM set to true.
            const bool status_has_pwm = true;             
            // For high to be on use 255 for the on state and 0 for the off, for low as on flip it.
            const uint8_t status_led_on_state = 0;//255;
            const uint8_t status_led_off_state = 4095;//0;  
                
            #if defined(ARDUINO_TEENSY41)    
                // SPI Follower Pins
                const unsigned int miso_pin ;
                const unsigned int mosi_pin= 11;
                const unsigned int sck_pin ;
                const unsigned int cs_pin = 10;
                const unsigned int irq_pin = 34;
                const unsigned int rst_pin = 4;
                const unsigned int spi_mode = 8; // This is 8 or 16 bit, not the actual SPI mode, I know it is confusing but that is how they chose to make the library.
                
                
                // Pin to Stop the Motors
                const unsigned int motor_stop_pin = 9;
                
                // Pin to use when we need a value but don't actually want to use it.
                const unsigned int not_connected_pin = 51;  // selected 51 as it is a pad on the back so I figure it won't hurt anything if something goes wrong.
                
                // Motor enable Pins
                const unsigned int enable_left_pin[] = {28, 29};
                const unsigned int enable_right_pin[] = {8, 7};
                
                
                const unsigned int speed_check_pin = 33;
				
				const unsigned int left_ankle_angle_pin = A13;
                const unsigned int right_ankle_angle_pin = A12;
                
                // I2C 
                // SDA 18
                // SCL 19
                
                //SPI
                // 
            #endif
            
            const unsigned int motor_enable_on_state = HIGH;
            const unsigned int motor_enable_off_state = LOW;
        };
        
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        namespace coms_micro_pins  //nano
        {
            const unsigned int blue = 24;
            const unsigned int green = 23;
            const unsigned int red = 22;
            const unsigned int led_active_low = 1;
            
            // SPI Conroller Pins
            const unsigned int miso_pin = 11;
            const unsigned int mosi_pin= 12;
            const unsigned int sck_pin = 13;
            const unsigned int cs_pin = 10;
            const unsigned int spi_mode = 16;
            
            
        };
        #endif
	#elif BOARD_VERSION == AK_Board_V0_6_Maxon 
       
        #include "Arduino.h"
        namespace logic_micro_pins  //teensy
        {
             #if defined(ARDUINO_TEENSY41)
                // Serial Pins, NC
                const unsigned int rx1_pin;
                const unsigned int tx1_pin;
                
                // CAN Pins
                const unsigned int can_rx_pin;
                const unsigned int can_tx_pin;
                
                // FSR Pins
                const unsigned int fsr_sense_left_heel_pin = A14;
                const unsigned int fsr_sense_left_toe_pin = A15;
                const unsigned int fsr_sense_right_heel_pin= A3;
                const unsigned int fsr_sense_right_toe_pin = A2;
                
                // Torque Sensor Pins
                const unsigned int num_available_joints = 2;
                //const unsigned int torque_sensor_left[] = {A16, A17};
				const unsigned int torque_sensor_left[] = {A16};
                //const unsigned int torque_sensor_left1 = A16;
                //const unsigned int torque_sensor_right[] = {A6, A7};
				const unsigned int torque_sensor_right[] = {A6};
                //const unsigned int torque_sensor_right1 = A8;
                
                
                // Sync LED Pins
                const unsigned int sync_led_pin ;
                const unsigned int sync_default_pin ;
            #endif
            // Arduino compiles all files not just the ones that are used so this is not under teensy to prevent errors
            const unsigned int sync_led_on_state = LOW;//HIGH;
            const unsigned int sync_led_off_state = HIGH;//LOW;

             #if defined(ARDUINO_TEENSY41)
                // Status LED Pins
                const unsigned int status_led_r_pin ;
                const unsigned int status_led_g_pin ;
                const unsigned int status_led_b_pin ;
            #endif

            // if you have connected to pins with PWM set to true.
            const bool status_has_pwm = true;             
            // For high to be on use 255 for the on state and 0 for the off, for low as on flip it.
            const uint8_t status_led_on_state = 0;//255;
            const uint8_t status_led_off_state = 4095;//0;  
                
            #if defined(ARDUINO_TEENSY41)    
                // SPI Follower Pins
                const unsigned int miso_pin ;
                const unsigned int mosi_pin ;
                const unsigned int sck_pin ;
                const unsigned int cs_pin ;
                const unsigned int irq_pin ;
                const unsigned int rst_pin ;
                const unsigned int spi_mode = 8; // This is 8 or 16 bit, not the actual SPI mode, I know it is confusing but that is how they chose to make the library.
                
                
                // Pin to Stop the Motors
                const unsigned int motor_stop_pin = 51;
                
                // Pin to use when we need a value but don't actually want to use it.
                const unsigned int not_connected_pin = 51;  // selected 51 as it is a pad on the back so I figure it won't hurt anything if something goes wrong.
                
                // Motor enable Pins
                const unsigned int enable_left_pin[] = {28, 29};
                const unsigned int enable_right_pin[] = {33, 33};
                
                
                const unsigned int speed_check_pin ;
				
				const unsigned int left_ankle_angle_pin;
                const unsigned int right_ankle_angle_pin;
                
                // I2C 
                // SDA 18
                // SCL 19
                
                //SPI
                // 
            #endif
            
            const unsigned int motor_enable_on_state = HIGH;
            const unsigned int motor_enable_off_state = LOW;
        };
        
        #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
        namespace coms_micro_pins  //nano
        {
            const unsigned int blue = 24;
            const unsigned int green = 23;
            const unsigned int red = 22;
            const unsigned int led_active_low = 1;
            
            // SPI Conroller Pins
            const unsigned int miso_pin = 11;
            const unsigned int mosi_pin= 12;
            const unsigned int sck_pin = 13;
            const unsigned int cs_pin = 10;
            const unsigned int spi_mode = 16;
        };
        #endif
    #endif 
#endif
