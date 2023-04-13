/*
   Code to test the Status LED
   P. Stegall April 2022
*/
#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)
  // general headers for using the system
  #include "src\Board.h"
  #include "src\Utilities.h"
  #include "src\ParseIni.h"
  
  // header for the component we are checking.
  #include "src\TorqueSensor.h"
  
  namespace config_info
  {
    uint8_t (config_to_send)[ini_config::number_of_keys];
  }
  
  void setup()
  {
    Serial.begin(115200);
    while(!Serial)
    {
      ;
    }

    logger::print("left_0");
    logger::print("\t");
    logger::print("left_1");
    logger::print("\t");
    logger::print("right_0");
    logger::print("\t");
    logger::print("right_1");
    logger::print("\n");

    
//    #if BOARD_VERSION == AK_Board_V0_1
//      logger::println("Board : AK_Board_V0_1");
//    #elif BOARD_VERSION == AK_Board_V0_3
//      logger::println("Board : AK_Board_V0_3");
//    #endif
//  
//    #if defined(ARDUINO_TEENSY36)
//      logger::println("Teensy 3.6");
//    #elif defined(ARDUINO_TEENSY41)
//      logger::println("Teensy 4.1");
//    #endif
//  
//    
//    
//    logger::print(logic_micro_pins::status_led_r_pin);
//    logger::print("\t");
//    logger::print(logic_micro_pins::status_led_g_pin);
//    logger::print("\t");
//    logger::print(logic_micro_pins::status_led_b_pin);
//    logger::print("\n");
    
  }
  
  void loop()
  {
    bool is_left = false;
    static TorqueSensor left_0_torque_sensor(logic_micro_pins::torque_sensor_left[0]);
    static TorqueSensor left_1_torque_sensor(logic_micro_pins::torque_sensor_left[1]);
    static TorqueSensor right_0_torque_sensor(logic_micro_pins::torque_sensor_right[0]);
    static TorqueSensor right_1_torque_sensor(logic_micro_pins::torque_sensor_right[1]);
      
    int state_period_ms = 1;
    static int last_transition_time = millis();
    int current_time = millis();
    


    static bool do_calibration_left_0 = true; //bit 0 is calibrate fsr, bit 1 is refine calibration.
    static bool do_calibration_left_1 = true; 
    static bool do_calibration_right_0 = true; //bit 0 is calibrate fsr, bit 1 is refine calibration.
    static bool do_calibration_right_1 = true;
    
    if (state_period_ms <= (current_time - last_transition_time))
    {
      do_calibration_left_0 = left_0_torque_sensor.calibrate(do_calibration_left_0);
      do_calibration_left_1 = left_1_torque_sensor.calibrate(do_calibration_left_1);
      do_calibration_right_0 = right_0_torque_sensor.calibrate(do_calibration_right_0);
      do_calibration_right_1 = right_1_torque_sensor.calibrate(do_calibration_right_1);

      float torque_reading_left_0 = left_0_torque_sensor.read();
      float torque_reading_left_1 = left_1_torque_sensor.read();
      float torque_reading_right_0 = right_0_torque_sensor.read();
      float torque_reading_right_1 = right_1_torque_sensor.read();
      
      last_transition_time = current_time;
      logger::print(torque_reading_left_0);
      logger::print("\t");
      logger::print(torque_reading_left_1);
      logger::print("\t");
      logger::print(torque_reading_right_0);
      logger::print("\t");
      logger::print(torque_reading_right_1);
      logger::print("\n");
      
    }
    
  }


#endif
