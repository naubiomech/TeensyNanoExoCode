#ifndef ERRORMANAGER_H
#define ERRORMANAGER_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "error_types.h"


/**
 * @brief Class manages the calling of error handlers and triggers. Only the control MCU is required to use this.
 * The triggers and handlers must be assigned before check is ran. They are defined in error_triggers.h and error_handlers.h.
 * Check should be called every loop, and should not, severely, increase the loop time. 
 */
class ErrorManager
{
    public:
        ErrorManager(Exo* exo, ExoData* exo_data);
        /**
         * @brief Checks for and reports errors.
         * 
         * @return bool True if there is a new error to report
         */
        bool check();
        /**
         * @brief Assigns callbacks for the handlers
         * 
         * @param soft Function to handle 'soft' errors
         * @param hard Function to handle 'hard' errors
         * @param fatal Function to handle 'fatal' errors
         */
        void assign_handlers(error_handler_t soft, error_handler_t hard, error_handler_t fatal);
        /**
         * @brief Assigns callbacks for the triggers
         * 
         * @param soft Function to trigger 'soft' errors
         * @param hard Function to trigger 'hard' errors
         * @param fatal Function to trigger 'fatal' errors
         */
        void assign_triggers(error_trigger_t soft, error_trigger_t hard, error_trigger_t fatal);
        /**
         * @brief Return the new error code
         * 
         * @return int 
         */
        int get_error();
        /**
         * @brief Generates an error. This is used to pass errors from other system modules.
         * 
         * @param to_code Error Code to pass
         */
        static void set_system_error(int to_code);

    private:
        /**
         * @brief Checks if there is a new error to report
         * 
         * @return true 
         * @return false 
         */
        bool _new_error(int working_error_code);

        Exo* _exo;
        ExoData* _data;

        bool _set_handlers;
        bool _set_triggers;

        error_handler_t _soft_handler = NULL;
        error_handler_t _hard_handler = NULL;
        error_handler_t _fatal_handler = NULL;

        error_trigger_t _soft_trigger = NULL;
        error_trigger_t _hard_trigger = NULL;
        error_trigger_t _fatal_trigger = NULL;

        static int _system_error_code; /* Used to pass errors from other system modules*/
        static int _reported_error_code; /* Used to pass errors from other system modules*/
};

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)