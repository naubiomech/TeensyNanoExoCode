/**
 * @file UARTHandler.h
 * @author Chance Cuddeback
 * @brief Singleton class that manages the UART data. NOT THREAD SAFE. The class queues recieved messages. 
 * @date 2022-09-07
 * 
 */


#ifndef UARTHandler_h
#define UARTHandler_h

//#include "Board.h"
//#include "ParseIni.h"
//#include "ExoData.h"
//#include "Utilities.h"
#include "UART_msg_t.h"

#include "Arduino.h"
#include <stdint.h>

#define MAX_NUM_LEGS 2
#define MAX_NUM_JOINTS_PER_LEG 2 // current PCB can only do 2 motors per leg.
#define MAX_RAW_BUFFER_SIZE 256
#define MAX_DATA_SIZE 32
// if type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define UART_DATA_TYPE short int
#define FIXED_POINT_FACTOR 100
#define UART_BAUD 256000

#define MAX_RX_LEN 64 // bytes
#define RX_TIMEOUT_US 1000

/* SLIP special character codes
*/
#define END             0300    /* indicates end of packet */
#define ESC             0333    /* indicates byte stuffing */
#define ESC_END         0334    /* ESC ESC_END means END data byte */
#define ESC_ESC         0335    /* ESC ESC_ESC means ESC data byte */

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
#define MY_SERIAL Serial8
#elif defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
#define MY_SERIAL Serial1
#else 
#error No Serial Object Found
#endif

/**
 * @brief Singleton Class to handle the UART Work. 
 * 
 */
class UARTHandler
{
    public:
        /**
         * @brief Get the instance object
         * 
         * @return UARTHandler* A reference to the singleton
         */
        static UARTHandler* get_instance();

        /**
         * @brief Packs and sends a UART message
         * 
         * @param msg_id An ID used to associate data on the receiver
         * @param len Length of data
         * @param joint_id Joint ID associated with data
         * @param buffer Payload
         */
        void UART_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *buffer);
        void UART_msg(UART_msg_t msg);

        /**
         * @brief Check for incoming data. If there is data read the message, timing out if it takes too long.
         * 
         * @param timeout_us 
         * @return UART_msg_t 
         */
        UART_msg_t poll(float timeout_us = RX_TIMEOUT_US);

        /**
         * @brief See if data is available in the UART buffer
         * 
         * @return uint8_t The ammount of bytes available in the UART buffer (max 64 for Arduino)
         */
        uint8_t check_for_data();

    private:
        /**
         * @brief Construct a new UARTHandler object
         * 
         */
        UARTHandler();

        void _pack(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data, uint8_t *data_to_pack);

        UART_msg_t _unpack(uint8_t* data, uint8_t len);

        uint8_t _get_packed_length(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data);

        void _send_packet(uint8_t* p, uint8_t len);

        int _recv_packet(uint8_t *p, uint8_t len = MAX_RX_LEN);

        void _send_char(uint8_t val);

        uint8_t _recv_char(void);

        uint8_t _time_left(uint8_t should_latch = 0);


        /* Data */
        //circular_buffer<uint8_t, 64> _rx_raw;

        float _timeout_us = RX_TIMEOUT_US;
        
        uint8_t _partial_packet[MAX_RX_LEN];
        uint8_t _partial_packet_len = 0;

};



#endif