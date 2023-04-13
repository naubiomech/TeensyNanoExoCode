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
#include "src\Board.h"
#include "src\ExoData.h"
#include "src\Exo.h"                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
#include "src\Utilities.h"
#include "src\StatusDefs.h"

// Specific Libraries
#include "src\ParseIni.h"
#include "src\ParamsFromSD.h"

// Board to board coms
#include "src\UARTHandler.h"
#include "src\uart_commands.h"
#include "src\UART_msg_t.h"

// Error Handling
#include "src\error_handlers.h"
#include "src\error_triggers.h"
#include "src\ErrorManager.h"

// Logging
#include "src\Logger.h"
#include "src\PiLogger.h"

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
 // TODO: Remove serial while for deployed version as this would hang
     //while (!Serial) {
      //; // wait for serial port to connect. Needed for native USB
     //}

    // get the config information from the SD card.
    ini_parser(config_info::config_to_send);

    // wait for the nano to get started
    //delay(500);
    
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

    static ErrorManager error_manager(&exo, &exo_data);
    static UARTHandler* uart_handler = UARTHandler::get_instance();
    
    if (first_run)
    {
        first_run = false;
        
        UART_command_utils::wait_for_get_config(uart_handler, &exo_data, UART_times::CONFIG_TIMEOUT);

        // assign the error handlers and triggers
        error_manager.assign_handlers(error_handlers::soft, error_handlers::hard, error_handlers::fatal);
        error_manager.assign_triggers(error_triggers::soft, error_triggers::hard, error_triggers::fatal);
      
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
    if (ran) 
    {
        //pi_logger.sendUpdate();
    }

    // manage system errors
    static bool reported_error{false};
    static bool new_error{false};
    static bool active_trial{false};
    uint16_t exo_status = exo_data.get_status();
    active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement) ||
        (exo_status == status_defs::messages::error);
    
    if (active_trial && ran && !exo_data.user_paused)
    {
        //new_error = error_manager.check();
    }
    
    if (new_error && !reported_error)
    {
        // Only report the first error
        reported_error = true;
        const int error_code = error_manager.get_error();
        
        exo_data.error_code = error_code;
        exo_data.set_status(status_defs::messages::error);
        UART_command_handlers::get_error_code(uart_handler, &exo_data, UART_msg_t());
    }
    
    // print the exo_data at a fixed period.
//    unsigned int data_print_ms = 5000;
//    static unsigned int last_data_time = millis();
//    if(millis() - last_data_time > data_print_ms)
//    {
//        logger::print("\n\n\nSuperloop :: Timed print : ");
//        exo_data.print();
//        last_data_time = millis();
//    }
    
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

// create array to store config.
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

void setup()
{
    logger::println();

    logger::print("Setup->Getting config");
    // get the sd card config from the teensy, this has a timeout
    UARTHandler* handler = UARTHandler::get_instance();
    bool timed_out = UART_command_utils::get_config(handler, config_info::config_to_send, (float)UART_times::CONFIG_TIMEOUT);

    ComsLed* led = ComsLed::get_instance();
    if (timed_out)
    {
        // yellow
        logger::print("Setup->Timed Out Getting Config", LogLevel::Warn);
        led->set_color(255, 255, 0);
    }
    else
    {
        // green
        led->set_color(0, 255, 0);
    }

    #if REAL_TIME_I2C
      real_time_i2c::init();
    #endif
    logger::print("Setup->End Setup");
}

void loop()
{
    static ExoData* exo_data = new ExoData(config_info::config_to_send);
    static ComsMCU* mcu = new ComsMCU(exo_data, config_info::config_to_send);
    static WaistBarometer* waist_barometer = new WaistBarometer();
    static InclineDetector* incline_detector = new InclineDetector();
    mcu->handle_ble();
    mcu->local_sample();
    mcu->update_UART();
    mcu->update_gui();
    mcu->handle_errors();

    static float then = millis();
    float now = millis();
    if ((now - then) > INCLINE_DELTA_MS)
    {
        then = now;
        incline_state_t state = incline_detector->run(waist_barometer->getPressure());
    }
}

#else // code to use when microcontroller is not recognized.
void setup()
{
  logger::print("Unknown Microcontroller");
  logger::print("\n");
}

void loop()
{

}


#endif
