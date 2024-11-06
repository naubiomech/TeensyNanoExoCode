/*
   Code used to run the exo from the teensy.  This communicates with the nano over UART.
   
   
   P. Stegall Jan 2022
*/  

#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)

//#define INCLUDE_FLEXCAN_DEBUG  // used to print CAN Debugging messages for the motors.
//#define MAKE_PLOTS  // Do prints for plotting when uncommented.
//#define MAIN_DEBUG  // Print Arduino debugging statements when uncommented.
//#define HEADLESS // used when there is no app access.

// Standard Libraries
#include <stdint.h>
#include <IntervalTimer.h>

// for the include files we can eventually create a library properties files but right now just providing the path should work.
// Common Libraries
#include "src/Board.h"
#include "src/ExoData.h"
#include "src/Exo.h"                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
#include "src/Utilities.h"
#include "src/StatusDefs.h"

// Specific Libraries
#include "src/ParseIni.h"
#include "src/ParamsFromSD.h"

// Board to board coms
#include "src/UARTHandler.h"
#include "src/uart_commands.h"
#include "src/UART_msg_t.h"

// Logging
#include "src/Logger.h"
#include "src/PiLogger.h"

//#include "src\Motor.h"

// Array used to store config information
namespace config_info
{
    uint8_t (config_to_send)[ini_config::number_of_keys];
}

void setup()
{
    //analogWriteResolution(12);
    analogReadResolution(12);
    
    Serial.begin(115200);
    delay(100);

    // get the config information from the SD card.
    ini_parser(config_info::config_to_send);
    
    // Print to confirm config came through correctly. Should not contain zeros.
    #ifdef MAIN_DEBUG
        for (int i = 0; i < ini_config::number_of_keys; i++)
        {
          logger::print("[" + String(i) + "] : " + String((int)config_info::config_to_send[i]) + "\n");
        }
        logger::print("\n");
    #endif
    
    // labels for the signals if plotting.
    #ifdef MAKE_PLOTS
          logger::print("Left_hip_trq_cmd, ");
          logger::print("Left_hip_current, ");
          logger::print("Right_hip_trq_cmd, ");
          logger::print("Right_hip_current, ");
          logger::print("Left_ankle_trq_cmd, ");
          logger::print("Left_ankle_current, ");
          logger::print("Right_ankle_trq_cmd, ");
          logger::print("Right_ankle_current, ");
          logger::print("Left_ankle_torque_measure, ");
          logger::print("\n");
      #endif
  
}



void loop()
{       
    static bool first_run = true;
    
    // create the data and exo objects
    static ExoData exo_data(config_info::config_to_send);

    #ifdef MAIN_DEBUG
        if (first_run)
        {
            logger::print("Superloop :: exo_data created"); 
        }
    #endif
    static Exo exo(&exo_data);
    #ifdef MAIN_DEBUG
        if (first_run)
        {
            logger::print("Superloop :: exo created");
        }
    #endif

    static UARTHandler* uart_handler = UARTHandler::get_instance();
    
    if (first_run)
    {
        first_run = false;
        
        UART_command_utils::wait_for_get_config(uart_handler, &exo_data, UART_times::CONFIG_TIMEOUT);

      
        #ifdef MAIN_DEBUG
            logger::print("Superloop :: Start First Run Conditional\n");
            logger::print("Superloop :: exo_data.left_leg.hip.is_used = ");
            logger::print(exo_data.left_leg.hip.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.right_leg.hip.is_used = ");
            logger::print(exo_data.right_leg.hip.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.left_leg.knee.is_used = ");
            logger::print(exo_data.left_leg.knee.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.right_leg.knee.is_used = ");
            logger::print(exo_data.right_leg.knee.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.left_leg.ankle.is_used = ");
            logger::print(exo_data.left_leg.ankle.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.right_leg.ankle.is_used = ");
            logger::print(exo_data.right_leg.ankle.is_used);
            logger::print("\n");
            logger::print("\n");
        #endif
        
        // debug to check the message is coming through
        //exo_data.left_leg.hip.motor.p_des = 300;
        
        // Only make calls to used motors.
        if (exo_data.left_leg.hip.is_used)
        {
            // turn motor on
            exo_data.left_leg.hip.motor.is_on = true;
            //exo.left_leg._hip._motor->on_off();
            // make sure gains are 0 so there is no funny business.
            exo_data.left_leg.hip.motor.kp = 0;
            exo_data.left_leg.hip.motor.kd = 0;
            #ifdef HEADLESS
                //exo.left_leg._hip._motor->zero();
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Hip Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.left_leg.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data); //This function is found in ParamsFromSD
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Hip Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.left_leg.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque; // start in zero torque
                exo.left_leg._hip.set_controller(exo_data.left_leg.hip.controller.controller); //set_controller is found in Joint
            #endif
        }
        
        if (exo_data.right_leg.hip.is_used)
        {
            exo_data.right_leg.hip.motor.is_on = true;
            //exo.right_leg._hip._motor->on_off();
            exo_data.right_leg.hip.motor.kp = 0;
            exo_data.right_leg.hip.motor.kd = 0;
            #ifdef HEADLESS
                //exo.right_leg._hip._motor->zero();
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Hip Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.right_leg.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data);
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Hip Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.right_leg.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque; // start in zero torque
                exo.right_leg._hip.set_controller(exo_data.right_leg.hip.controller.controller);
            #endif
        }

        if (exo_data.left_leg.ankle.is_used)
        {
            #ifdef MAIN_DEBUG
              logger::print("Superloop :: Left Ankle Used");
            #endif
            exo_data.left_leg.ankle.motor.is_on = true;
            //exo.left_leg._ankle._motor->on_off();
            exo_data.left_leg.ankle.motor.kp = 0;
            exo_data.left_leg.ankle.motor.kd = 0;
            #ifdef HEADLESS
                //exo.left_leg._ankle._motor->zero();
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Ankle Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.left_leg.ankle.id, config_info::config_to_send[config_defs::exo_ankle_default_controller_idx], 0, &exo_data);
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Ankle Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.left_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque; // start in zero torque
                exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);              
            #endif
        }
        
        if (exo_data.right_leg.ankle.is_used)
        {
            exo_data.right_leg.ankle.motor.is_on = true;
            //exo.right_leg._ankle._motor->on_off();
            exo_data.right_leg.ankle.motor.kp = 0;
            exo_data.right_leg.ankle.motor.kd = 0;
            #ifdef HEADLESS
                //exo.right_leg._ankle._motor->zero();
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Ankle Zeroed");
                #endif
                
                set_controller_params((uint8_t) exo_data.right_leg.ankle.id, config_info::config_to_send[config_defs::exo_ankle_default_controller_idx], 0, &exo_data);
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Ankle Parameters Set");
                #endif
                
                //wait till calibration is done to set actual controller
                exo_data.right_leg.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque; // start in zero torque
                exo.right_leg._ankle.set_controller(exo_data.right_leg.ankle.controller.controller);
            #endif
        }
        
        // give the motors time to wake up.  Can eventually be removed when using non damaged motors.
        #ifdef MAIN_DEBUG
          logger::print("Superloop :: Motor Charging Delay - Please be patient");
        #endif 
        exo_data.set_status(status_defs::messages::motor_start_up); 
        unsigned int motor_start_delay_ms = 10;//60000;
        unsigned int motor_start_time = millis();
        unsigned int dot_print_ms = 1000;
        unsigned int last_dot_time = millis();
        while (millis() - motor_start_time < motor_start_delay_ms)
        {
            exo.status_led.update(exo_data.get_status());
            #ifdef MAIN_DEBUG
              if(millis() - last_dot_time > dot_print_ms)
              {
                last_dot_time = millis();
                logger::print(".");
              }
              
            #endif
        }
        #ifdef MAIN_DEBUG
          logger::println();
        #endif

        // Configure the system if you can't set it with the app
        #ifdef HEADLESS
            bool enable_overide = true;
            if(exo_data.left_leg.hip.is_used)
            {
                exo_data.left_leg.hip.calibrate_torque_sensor = true; 
                exo_data.left_leg.hip.motor.enabled = true;
                //exo.left_leg._hip._motor->enable(enable_overide);
            }
           
            if(exo_data.right_leg.hip.is_used)
            {
                exo_data.right_leg.hip.calibrate_torque_sensor = true; //Flag in JointData
                exo_data.right_leg.hip.motor.enabled = true; //Flag in MotorData
                //exo.right_leg._hip._motor->enable(enable_overide);
            }
            if(exo_data.left_leg.ankle.is_used)
            {
                exo_data.left_leg.ankle.calibrate_torque_sensor = true; 
                exo_data.left_leg.ankle.motor.enabled = true;
                //exo.left_leg._ankle._motor->enable(enable_overide);
            }
           
            if(exo_data.right_leg.ankle.is_used)
            {
                exo_data.right_leg.ankle.calibrate_torque_sensor = true;  
                exo_data.right_leg.ankle.motor.enabled = true;
                //exo.right_leg._ankle._motor->enable(enable_overide);
            }
        #endif
 
        #ifdef MAIN_DEBUG
            #ifdef HEADLESS
                logger::print("Superloop :: Motors Enabled");
                logger::print("Superloop :: Parameters Set");
            #endif
            logger::print("Superloop :: End First Run Conditional");
        #endif
    }

    // run the calibrations we need to do if not set through the app.
    #ifdef HEADLESS
        static bool static_calibration_done = false;
        unsigned int pause_after_static_calibration_ms = 10000;
        static unsigned int time_dynamic_calibration_finished; 
        static bool pause_between_calibration_done = false;   
        static bool dynamic_calibration_done = false;
    
        
        // do data plotting
        static float old_time = micros();
        float new_time = micros();
        if(new_time - old_time > 10000 && dynamic_calibration_done)
        {
            #ifdef MAKE_PLOTS
//                logger::print(exo_data.left_leg.hip.motor.t_ff);
//                logger::print(", ");
//                logger::print(exo_data.left_leg.hip.motor.i);
//                logger::print(", ");
//                logger::print(exo_data.right_leg.hip.motor.t_ff);
//                logger::print(", ");
//                logger::print(exo_data.right_leg.hip.motor.i);
//                logger::print(", ");
//                logger::print(exo_data.left_leg.ankle.motor.t_ff);
                //logger::print(", ");
                //logger::print(exo_data.right_leg.hip.motor.i);
//                logger::print(", ");
//                logger::print(exo_data.right_leg.ankle.motor.t_ff);
//                logger::print(", ");
//                logger::print(exo_data.right_leg.ankle.motor.i);
                //logger::print(", ");
                //logger::print(exo_data.right_leg.hip.torque_reading);
                //logger::print("\n");
            #endif

            old_time = new_time;
            
        }
    
        
        // do torque sensor calibration
        if ((!static_calibration_done) && (!exo_data.left_leg.ankle.calibrate_torque_sensor && !exo_data.right_leg.ankle.calibrate_torque_sensor))
        {
            #ifdef MAIN_DEBUG
              logger::print("Superloop : Static Calibration Done");
            #endif
            static_calibration_done  = true;
            time_dynamic_calibration_finished = millis();
            exo_data.set_status(status_defs::messages::test);
        }
    
        // pause between static and dynamic calibration so we have time to start walking
        if (!pause_between_calibration_done && (static_calibration_done && ((time_dynamic_calibration_finished +  pause_after_static_calibration_ms) < millis() ))) 
        {
            #ifdef MAIN_DEBUG
              logger::print("Superloop : Pause Between Calibration Finished");
            #endif
            if(exo_data.left_leg.is_used)
            {
                exo_data.left_leg.do_calibration_toe_fsr = true;              //Flag in LegData
                exo_data.left_leg.do_calibration_refinement_toe_fsr = true;   //Flag in LegData
                exo_data.left_leg.do_calibration_heel_fsr = true;             //Flag in LegData
                exo_data.left_leg.do_calibration_refinement_heel_fsr = true;  //Flag in LegData
            }
           
            if(exo_data.right_leg.is_used)
            {
                exo_data.right_leg.do_calibration_toe_fsr = true;
                exo_data.right_leg.do_calibration_refinement_toe_fsr = true;
                exo_data.right_leg.do_calibration_heel_fsr = true;
                exo_data.right_leg.do_calibration_refinement_heel_fsr = true;
            }
            pause_between_calibration_done = true;
        }
            
        // do the dynamic calibrations
        if ((!dynamic_calibration_done) && (pause_between_calibration_done) && (!exo_data.left_leg.do_calibration_toe_fsr && !exo_data.left_leg.do_calibration_refinement_toe_fsr && !exo_data.left_leg.do_calibration_heel_fsr && !exo_data.left_leg.do_calibration_refinement_heel_fsr))
        {
            #ifdef MAIN_DEBUG
                logger::print("Superloop : Dynamic Calibration Done");
            #endif
            
            if (exo_data.left_leg.hip.is_used)
            {
                exo_data.left_leg.hip.controller.controller = config_info::config_to_send[config_defs::exo_hip_default_controller_idx];
                exo.left_leg._hip.set_controller(exo_data.left_leg.hip.controller.controller);
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Left Hip Controller Set");
                #endif
            }
            
            if (exo_data.right_leg.hip.is_used)
            {
                exo_data.right_leg.hip.controller.controller = config_info::config_to_send[config_defs::exo_hip_default_controller_idx];
                exo.right_leg._hip.set_controller(exo_data.right_leg.hip.controller.controller); 
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Right Hip Controller Set");
                #endif
            }
            
            if (exo_data.left_leg.ankle.is_used)
            {
                exo_data.left_leg.ankle.controller.controller = config_info::config_to_send[config_defs::exo_ankle_default_controller_idx];
                exo.left_leg._ankle.set_controller(exo_data.left_leg.ankle.controller.controller);
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Left Ankle Controller Set");
                #endif
            }
      
            if (exo_data.right_leg.ankle.is_used)
            {
                exo_data.right_leg.ankle.controller.controller = config_info::config_to_send[config_defs::exo_ankle_default_controller_idx];
                exo.right_leg._ankle.set_controller(exo_data.right_leg.ankle.controller.controller);
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Right Ankle Controller Set");
                #endif
            }
            
            dynamic_calibration_done = true;
          
        }
    #endif                                                                                        

    // do exo calculations
    bool ran = exo.run();
    
    // Print some dots so we know it is doing something
    #ifdef MAIN_DEBUG
        unsigned int dot_print_ms = 5000;
        static unsigned int last_dot_time = millis();
        if(millis() - last_dot_time > dot_print_ms)
        {
          last_dot_time = millis();
          logger::print(".");
        }
    #endif 
}


#elif defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)  // board name is ARDUINO_[build.board] property in the board.txt file here found at C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\hardware\mbed_nano\2.6.1  They just already prepended it with ARDUINO so you have to do it twice.
#include <stdint.h>
#include "src/ParseIni.h"
#include "src/ExoData.h"
#include "src/ComsMCU.h"
#include "src/Config.h"
#include "src/Utilities.h"

// Board to board coms
#include "src/UARTHandler.h"
#include "src/uart_commands.h"
#include "src/UART_msg_t.h"
#include "src/ComsLed.h"
#include "src/RealTimeI2C.h"

#include "src/WaistBarometer.h"
#include "src/InclineDetector.h"

#define MAIN_DEBUG 0

// create array to store config.
namespace config_info
{
     uint8_t config_to_send[ini_config::number_of_keys] = {
            1,  //Board name
            3,  //Board version
            2,  //Battery
            22,  //Exo name
            3,  //Exo side
            5,  //Hip
            5,  //Knee
            5,  //Ankle
            4,  //Hip gear
            4,  //Knee gear
            4,  //Ankle gear
            6,  //Hip default controller
            6,  //Knee default controller
            10, //Ankle default controller
            6,  //Elbow default controller
            2,  //Hip use torque sensor
            2,  //Knee use torque sensor
            2,  //Ankle use torque sensor
            2,  //Elbow use torque sensor
            4,  //Hip flip motor dir
            4,  //Knee flip motor dir
            4,  //Ankle flip motor dir
            4,  //Elbow flip motor dir
            4,  //Hip flip torque dir
            4,  //Knee flip torque dir
            4,  //Ankle flip torque dir
            4,  //Elbow flip torque dir
            4,  //Hip flip angle dir
            4,  //Knee flip angle dir
            4,  //Ankle flip angle dir
            4,  //Elbow flip angle dir
          };
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    #if MAIN_DEBUG
    while (!Serial);
    logger::print("Setup->Getting config");
    #endif
    // get the sd card config from the teensy, this has a timeout
    UARTHandler* handler = UARTHandler::get_instance();
    const bool timed_out = UART_command_utils::get_config(handler, config_info::config_to_send, (float)UART_times::CONFIG_TIMEOUT);
    
    ComsLed* led = ComsLed::get_instance();
    if (timed_out)
    {
        // yellow
        #if MAIN_DEBUG
        logger::print("Setup->Timed Out Getting Config", LogLevel::Warn);
        #endif
        led->set_color(255, 255, 0);
    }
    else
    {
        // green
        led->set_color(0, 255, 0);
    }

    #if REAL_TIME_I2C
    logger::print("Init I2C");  
    real_time_i2c::init();
    logger::print("Setup->End Setup");
    #endif
}

void loop()
{
    #if MAIN_DEBUG
    static bool first_run = true;
    if (first_run)
    {
      logger::println("Start Loop");
    }
    #endif
    static ExoData* exo_data = new ExoData(config_info::config_to_send);
    #if MAIN_DEBUG
    if (first_run)
    {
      logger::println("Construced exo_data");
    }
    #endif
    static ComsMCU* mcu = new ComsMCU(exo_data, config_info::config_to_send);
    #if MAIN_DEBUG
    if (first_run)
    {
      logger::println("Construced mcu");
    }
    #endif
    
    mcu->handle_ble();
    mcu->local_sample();
    mcu->update_UART();
    mcu->update_gui();
    mcu->handle_errors();

    #if MAIN_DEBUG
    static float then = millis();
    float now = millis();
    if ((now - then) > 1000)
    {
        then = now;
        logger::println("...");
    }
    first_run = false;
    #endif
    
}

#else // code to use when microcontroller is not recognized.
#include "Utilities.h"
void setup()
{
  Serial.begin(115200);
  utils::spin_on_error_with("Unknown Microcontroller");
}

void loop()
{

}


#endif
