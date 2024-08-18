/**
 * @file ParseIni.h
 *
 * @brief Declares the functions needed and defines mapping between the INI keys and the exo components 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef ParseIni_h
#define ParseIni_h

//Used for uint8_t
#include <stdint.h>

/**
 * @brief define the constants to use for the various arrays.
 */
namespace ini_config
{
    const int buffer_length = 500;  /**< length of the buffer for reading the file. */
    const int key_length = 25;      /**< Max length of the key name */
    const int section_length = 10;  /**< Max length of the section name */
    const int number_of_keys = 17;  /**< Number of keys to be parsed. */
}

//Includes for reading the ini file from the SD card.
//1 is the lowest value to confirm that data is present for sending over SPI

/**
 * @brief Namespace that defines numeric coding for different keys values. These are used throughout the code.
 */
namespace config_defs
{
    enum class board_name : uint8_t
    {
        AK_board = 1,
    };
    
    enum class board_version : uint8_t
    { 
        zero_one = 1,
        zero_three = 2,
        zero_four = 3,
        zero_five_one = 4,
    };
    
    enum class battery : uint8_t
    { 
        smart = 1,
        dumb = 2,
    };

    enum class exo_name : uint8_t
    { 
        bilateral_ankle = 1, 
        bilateral_hip = 2, 
        bilateral_hip_ankle = 3,
        left_ankle = 4,
        right_ankle = 5,
        left_hip = 6,
        right_hip = 7,
        left_hip_ankle = 8,
        right_hip_ankle = 9,
        test = 10,
        right_knee = 11,
    };
    
    enum class exo_side : uint8_t
    { 
        bilateral = 1, 
        left = 2, 
        right = 3,
    };
    
    enum class JointType
    {
        hip = 1,
        knee = 2,
        ankle = 3,
    };
    
    enum class motor : uint8_t
    { 
        not_used = 1, 
        AK60 = 2, 
        AK80 = 3,
        AK60_v1_1 = 4,
        AK60_v1_1_T = 5,
        AK70 = 6,
    };
    
    enum class gearing : uint8_t
    { 
        gearing_1_1 = 1,
        gearing_2_1 = 2,
        gearing_3_1 = 3,
        gearing_4_5_1 = 4,
    };
    
    
    enum class joint_id : uint8_t
    {
        // byte format : [0, is_left, !is_left, unused_joint, unused_joint, is_ankle, is_knee, is_hip]
        left = 0b01000000,
        right = 0b00100000,
        
        hip = 0b00000001,
        knee = 0b00000010,
        ankle = 0b00000100,
                
        left_hip = left|hip,
        left_knee = left|knee,
        left_ankle = left|ankle,
        
        right_hip = right|hip,
        right_knee = right|knee,
        right_ankle = right|ankle,
    };
        
    enum class  hip_controllers : uint8_t
    {
        disabled = 1,
        zero_torque = 2,
        heel_toe = 3,
        franks_collins_hip = 4,
        stasis = 5,
        constant_torque = 6,
        ptb_general = 7,
        hip_resist = 8,
        chirp = 9,
        step = 10,
    };
    
    enum class knee_controllers : uint8_t
    {
        disabled = 1,
        zero_torque = 2,
        stasis = 3,
        constant_torque = 4,
        elbow_min_max = 5,
        chirp = 6,
        step = 7,
    };
        
    enum class ankle_controllers : uint8_t
    {
        disabled = 1, 
        zero_torque = 2, 
        pjmc = 3,
        zhang_collins = 4,
        stasis = 5,
        constant_torque = 6,
        ptb_general = 7,
        gasp = 8,
		elbow_min_max = 9,
		calibr_manager = 10,
        chirp = 11,
        step = 12,
    };
    
    enum class flip_dir : uint8_t
    {
        neither = 1, 
        left = 2, 
        right = 3,
        both = 4,
    };
    
    static const int board_name_idx = 0;
    static const int board_version_idx = 1;
    
    static const int battery_idx = 2;
    
    static const int exo_name_idx = 3;
    static const int exo_side_idx = 4;
    
    static const int hip_idx = 5;
    static const int knee_idx = 6;
    static const int ankle_idx = 7;
    
    static const int hip_gear_idx = 8;
    static const int knee_gear_idx = 9;
    static const int ankle_gear_idx = 10;
    
    static const int exo_hip_default_controller_idx = 11;
    static const int exo_knee_default_controller_idx = 12;
    static const int exo_ankle_default_controller_idx = 13;
    
    static const int hip_flip_dir_idx = 14;
    static const int knee_flip_dir_idx = 15;
    static const int ankle_flip_dir_idx = 16;
	
	static const int hip_flip_motor_dir_idx = 17;
	static const int knee_flip_motor_dir_idx = 18;
	static const int ankle_flip_motor_dir_idx = 19;
	
	static const int hip_flip_torque_dir_idx = 20;
	static const int knee_flip_torque_dir_idx = 21;
	static const int ankle_flip_torque_dir_idx = 22;
	
	static const int hip_flip_angle_dir_idx = 23;
	static const int knee_flip_angle_dir_idx = 24;
	static const int ankle_flip_angle_dir_idx = 25;
	
	static const int left_hip_RoM_idx = 26;
	static const int right_hip_RoM_idx = 27;
	static const int left_knee_RoM_idx = 28;
	static const int right_knee_RoM_idx = 29;
	static const int left_ankle_RoM_idx = 30;
	static const int right_ankle_RoM_idx = 31;
}

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
    #include <SD.h>
    #include <SPI.h>
    #include "IniFile.h"

    //Need to install ArduinoSTL in the library manager to use map
    #include <map>
    #include <string>

    //The select pin used for the SD card
    #ifndef SD_SELECT
        #define SD_SELECT BUILTIN_SDCARD
    #endif

    /**
     * @brief Parses the config.ini file in the root folder of the SD card and puts the parsed data to the provided array
     * 
     * @param pointer to the uint8_t array to be updated with the encoded parameter info. Array should be ini_config::number_of_keys in length
     */
    void ini_parser(uint8_t* config_to_send); // uses default filename
    
    /**
     * @brief Parses the specified filename from the SD card and puts the parsed data to the array provided
     * 
     * @param pointer to the character array that contains a nonstandard filename to parse.
     * @param pointer to the uint8_t array to be updated with the encoded parameter info. Array should be ini_config::number_of_keys in length
     */
    void ini_parser(char* filename, uint8_t* config_to_send); //uses sent filename
    
    /**
     * @brief retrieve the key values and get the print the output
     *
     * @param pointer to char array that contains the section containing the key
     * @param pointer to char array that contains the key name
     * @param pointer to char array to store the key value
     * @param length of the buffer for the key value
     */
    void get_section_key(IniFile ini, const char* section, const char* key, char* buffer, size_t buffer_len);  
    
    /**
     * @brief Prints the error messages of IniFile object.
     * e is the error message, and eol true means a println will be used at the end.
     * Requires that Serial is defined.
     * 
     * @param error id
     * @param if true will print and end of line character
     */
    void ini_print_error_message(uint8_t e, bool eol = true); 

    /**
     * @brief Mappings of config names to uint8_t that will be sent to the nano
     * "0" is mapped to one to help with debug as we should never send all zeros in the bytes.
     * If you see a uint8_t that is zero it indicates the field didn't exist.
    */
    namespace config_map
    {  
        
        //Define our own type so we don't have to type so much
        typedef std::map<std::string, uint8_t> IniKeyCode;
        
        const IniKeyCode board_name = {{"AK_Board", (uint8_t)config_defs::board_name::AK_board},};
        const IniKeyCode board_version = 
        { 
            {"0.1", (uint8_t)config_defs::board_version::zero_one},
            {"0.3", (uint8_t)config_defs::board_version::zero_three},
            {"0.5.1", (uint8_t)config_defs::board_version::zero_five_one},
        };
        
        const IniKeyCode battery = 
        { 
            {"smart", (uint8_t)config_defs::battery::smart},
            {"dumb", (uint8_t)config_defs::battery::dumb},        
        };

        const IniKeyCode exo_name 
        { 
            {"bilateralAnkle", (uint8_t)config_defs::exo_name::bilateral_ankle}, 
            {"bilateralHip", (uint8_t)config_defs::exo_name::bilateral_hip}, 
            {"bilateralHipAnkle", (uint8_t)config_defs::exo_name::bilateral_hip_ankle},
            {"leftAnkle", (uint8_t)config_defs::exo_name::left_ankle},
            {"rightAnkle", (uint8_t)config_defs::exo_name::right_ankle},
            {"leftHip", (uint8_t)config_defs::exo_name::left_hip},
            {"rightHip", (uint8_t)config_defs::exo_name::right_hip},
            {"leftHipAnkle", (uint8_t)config_defs::exo_name::left_hip_ankle},
            {"rightHipAnkle", (uint8_t)config_defs::exo_name::right_hip_ankle},
            {"test", (uint8_t)config_defs::exo_name::test},
            {"rightKnee", (uint8_t)config_defs::exo_name::right_knee},
        };
        
        const IniKeyCode exo_side 
        { 
            {"bilateral", (uint8_t)config_defs::exo_side::bilateral}, 
            {"left", (uint8_t)config_defs::exo_side::left}, 
            {"right", (uint8_t)config_defs::exo_side::right},
        };
        
        const IniKeyCode motor 
        { 
            {"0", (uint8_t)config_defs::motor::not_used}, 
            {"AK60", (uint8_t)config_defs::motor::AK60}, 
            {"AK80", (uint8_t)config_defs::motor::AK80},
            {"AK60v1.1", (uint8_t)config_defs::motor::AK60_v1_1},
            {"AK60v1.1T", (uint8_t)config_defs::motor::AK60_v1_1_T},
            {"AK70", (uint8_t)config_defs::motor::AK70},
        };
        
        const IniKeyCode gearing 
        { 
            {"1", (uint8_t)config_defs::gearing::gearing_1_1}, 
            {"2", (uint8_t)config_defs::gearing::gearing_2_1}, 
            {"3", (uint8_t)config_defs::gearing::gearing_3_1}, 
            {"4.5", (uint8_t)config_defs::gearing::gearing_4_5_1},
        };
        
        
        const IniKeyCode hip_controllers 
        { 
            {"0", (uint8_t)config_defs::hip_controllers::disabled}, 
            {"zeroTorque", (uint8_t)config_defs::hip_controllers::zero_torque}, 
            {"heelToe", (uint8_t)config_defs::hip_controllers::heel_toe},
            {"franksCollinsHip", (uint8_t)config_defs::hip_controllers::franks_collins_hip},
            {"stasis", (uint8_t)config_defs::hip_controllers::stasis},
            {"constantTorque", (uint8_t)config_defs::hip_controllers::constant_torque},
            {"ptbGeneral", (uint8_t)config_defs::hip_controllers::ptb_general},
            {"hipResist", (uint8_t)config_defs::hip_controllers::hip_resist},
            {"chirp", (uint8_t)config_defs::hip_controllers::chirp},
            {"step", (uint8_t)config_defs::hip_controllers::step},

        };
        
        const IniKeyCode knee_controllers 
        { 
            {"0", (uint8_t)config_defs::knee_controllers::disabled}, 
            {"zeroTorque", (uint8_t)config_defs::knee_controllers::zero_torque}, 
            {"stasis", (uint8_t)config_defs::knee_controllers::stasis},
            {"constantTorque", (uint8_t)config_defs::knee_controllers::constant_torque},
            {"elbowMinMax", (uint8_t)config_defs::knee_controllers::elbow_min_max},
            {"chirp", (uint8_t)config_defs::knee_controllers::chirp},
            {"step", (uint8_t)config_defs::knee_controllers::step},
        };
        
        const IniKeyCode ankle_controllers 
        { 
            {"0", (uint8_t)config_defs::ankle_controllers::disabled}, 
            {"zeroTorque", (uint8_t)config_defs::ankle_controllers::zero_torque}, 
            {"PJMC", (uint8_t)config_defs::ankle_controllers::pjmc},
            {"zhangCollins", (uint8_t)config_defs::ankle_controllers::zhang_collins},
            {"stasis", (uint8_t)config_defs::ankle_controllers::stasis},
            {"constantTorque", (uint8_t)config_defs::ankle_controllers::constant_torque},
            {"ptbGeneral", (uint8_t)config_defs::ankle_controllers::ptb_general},
            {"GAsP", (uint8_t)config_defs::ankle_controllers::gasp},
			{"elbowMinMax", (uint8_t)config_defs::ankle_controllers::elbow_min_max},
			{"calibrManager", (uint8_t)config_defs::ankle_controllers::calibr_manager},
            {"chirp", (uint8_t)config_defs::ankle_controllers::chirp},
            {"step", (uint8_t)config_defs::ankle_controllers::step},
        };  
        
        const IniKeyCode flip_dir 
        { 
            {"0", (uint8_t)config_defs::flip_dir::neither}, 
            {"left", (uint8_t)config_defs::flip_dir::left}, 
            {"right", (uint8_t)config_defs::flip_dir::right},
            {"both", (uint8_t)config_defs::flip_dir::both},
            
        }; 
    };

    /**
     * @brief Holds the raw key value strings from the ini file
     */
    struct ConfigData{
        
        std::string board_name;
        std::string board_version;
        
        std::string battery;
        
        std::string exo_name;
        std::string exo_sides;
        
        std::string exo_hip;
        std::string exo_knee;
        std::string exo_ankle;
        
        std::string hip_gearing;
        std::string knee_gearing;
        std::string ankle_gearing;
        
        std::string exo_hip_default_controller;
        std::string exo_knee_default_controller;
        std::string exo_ankle_default_controller;
        
        std::string hip_flip_dir;
        std::string knee_flip_dir;
        std::string ankle_flip_dir;
		
		std::string hip_flip_motor_dir;
        std::string knee_flip_motor_dir;
        std::string ankle_flip_motor_dir;
		
		std::string hip_flip_torque_dir;
        std::string knee_flip_torque_dir;
        std::string ankle_flip_torque_dir;
		
		std::string hip_flip_angle_dir;
        std::string knee_flip_angle_dir;
        std::string ankle_flip_angle_dir;
		
		float left_hip_RoM;
		float right_hip_RoM;
		float left_knee_RoM;
		float right_knee_RoM;
		float left_ankle_RoM;
		float right_ankle_RoM;
		
    };
#endif


#endif