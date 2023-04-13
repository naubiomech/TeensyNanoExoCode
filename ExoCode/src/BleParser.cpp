
// #if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
#include "BleParser.h"
#include "ble_commands.h"
#include "Utilities.h"
#include "BleMessageQueue.h"
#include "Logger.h"
#include <vector>

BleParser::BleParser()
{
  ;
}

BleMessage* BleParser::handle_raw_data(char* buffer, int length)
{
    static BleMessage* return_msg = new BleMessage();
    if (!_waiting_for_data)
    {
        _handle_command(*buffer);
        if (_working_message.is_complete)
        {   
            return_msg->copy(&_working_message);
            _working_message.clear();
            _waiting_for_data = false;
        }
    }
    else
    {
        memcpy(&_buffer[_bytes_collected],buffer,length);
        _bytes_collected += length;
        if (_bytes_collected == _working_message.expecting*8)
        {
            return_msg->copy(&_working_message);
            for (int i=0;i<(_working_message.expecting*8);i+=8)
            {
                double f_tmp = 0;
                memcpy(&f_tmp,&_buffer[i],8);
                return_msg->data[i/8] = (float)f_tmp;
            }
            return_msg->is_complete = true;
            _working_message.clear();
            _waiting_for_data = false;
            _bytes_collected = 0;
        }
        else if (_bytes_collected >= _working_message.expecting*8)
        {
            _working_message.clear();
            _waiting_for_data = false;
            _bytes_collected = 0;
        }
    }
    return return_msg;
}

int BleParser::package_raw_data(byte* buffer, BleMessage &msg)
{
    //6 max characters can transmit -XXXXX, or XXXXXX
    static int maxChars = 12;
    //Size must be declared at initialization because of itoa()
    char cBuffer[maxChars];
    int buffer_index = 0;
    buffer[buffer_index++] = 'S';
    buffer[buffer_index++] = msg.command;
    itoa(msg.expecting, &cBuffer[0], 10);
    memcpy(&buffer[buffer_index++], &cBuffer[0], 1);
    buffer[buffer_index++] = 'c';
    for (int i = 0; i < msg.expecting; i++)
    {
        double data_to_send = (double)msg.data[i];
        //Send as Int to reduce bytes being sent
        int modData = int(data_to_send * 100);
        int cLength = utils::get_char_length(modData);
        if (cLength > maxChars)
        {
        cLength = 1;
        modData = 0;
        }
        //Populates cBuffer with a base 10 number
        itoa(modData, &cBuffer[0], 10);
        //Writes cLength indices of cBuffer into buffer
        memcpy(&buffer[buffer_index], &cBuffer[0], cLength);
        buffer_index += cLength;
        buffer[buffer_index++] = 'n';
    }
    return buffer_index;
}

/*
 * Private Functions
 */

void BleParser::_handle_command(char command)
{
    int length = -1;
    //Get the ammount of characters to wait for
    for(unsigned int i=0; i < sizeof(ble::commands)/sizeof(ble::commands[0]); i++)
    {
        if(command == ble::commands[i].command)
        {
            length = ble::commands[i].length;
            break;
        }
    }
    if (length == -1)
    {
        // Didnt find command in list
        _working_message.clear();
        //logger::println("BleParser::_handle_command->Command is not in list");
    }
    else
    {
        _waiting_for_data = (length != 0);
        _working_message.command = command;
        _working_message.expecting = length;
        _working_message.is_complete = !_waiting_for_data;
    }
}
// #endif