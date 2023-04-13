#include "UARTHandler.h"
#include "Utilities.h"
#include "Logger.h"

#define MAX_NUM_LEGS 2
#define MAX_NUM_JOINTS_PER_LEG 2 // current PCB can only do 2 motors per leg.
// if type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define UART_DATA_TYPE short int
#define FIXED_POINT_FACTOR 100

typedef enum 
{
  COMMAND = 0,
  JOINT_ID = 1,
  DATA_START = 2
} UARTPackingIndex;


UARTHandler::UARTHandler()
{
  //_rx_raw = CircularBuffer_<char>(_rx_raw_buffer, _k_bufferSize);
  MY_SERIAL.begin(UART_BAUD);
  MY_SERIAL.setTimeout(0);
}

UARTHandler* UARTHandler::get_instance()
{
    static UARTHandler* instance = new UARTHandler();
    return instance;
}

void UARTHandler::UART_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *buffer)
{
    uint8_t _packed_len = _get_packed_length(msg_id, len, joint_id, buffer);
  //  logger::print("UARTHandler::UART_msg->Packing Bytes: "); logger::println(_packed_len);
    uint8_t _byte_data[_packed_len] = {0};
    _pack(msg_id, len, joint_id, buffer, _byte_data);
  //  logger::println("UARTHandler::UART_msg->Packed data:");
  //  for (int i=0; i<_packed_len; i++)
  //  {
  //    logger::print(_byte_data[i], HEX); logger::print(", ");
  //  }
  //  logger::println();
    _send_packet(_byte_data, _packed_len);
    MY_SERIAL.flush();
  //  logger::println("UARTHandler::UART_msg->Flushed tx buffer");
}

void UARTHandler::UART_msg(UART_msg_t msg)
{
     //logger::print("UARTHandler::UART_msg->Sending Message");
     //UART_msg_t_utils::print_msg(msg);
    UART_msg(msg.command, msg.len, msg.joint_id, msg.data);
}

UART_msg_t UARTHandler::poll(float timeout_us)
{
    static UART_msg_t empty_msg = {0, 0, 0, 0};
    _timeout_us = timeout_us;
    
    uint32_t _available_bytes = check_for_data();
    if (!_available_bytes) {return empty_msg;}
    //logger::print("UARTHandler::poll->Bytes Available: "); logger::println(_available_bytes);

    uint8_t _msg_buffer[MAX_RX_LEN];
    uint8_t _recv_len = _recv_packet(_msg_buffer, MAX_RX_LEN);
    if (_recv_len > 0)
    {
      if (_partial_packet_len) 
      {
      //  logger::println("UARTHandler::poll->Packet requires shifting");
        // this occurs if there was a timeout during _recv_packet and we have a new partial packet
        // shift _msg_buffer _partial_packet_len bytes to fit the previous partial packet using memmove
        memmove(_msg_buffer + _partial_packet_len, _msg_buffer, _recv_len);
        memcpy(_msg_buffer, _partial_packet, _partial_packet_len);
        _recv_len += _partial_packet_len;

      //  memmove(_msg_buffer + (_partial_packet_len-1), _msg_buffer, _recv_len);

      //  // append the _partial packet to the full message
      //  memcpy(_msg_buffer, _partial_packet, _partial_packet_len);
       
       _partial_packet_len = 0;
     }

      UART_msg_t msg = _unpack(_msg_buffer, _recv_len);
      // logger::print("UARTHandler::poll->Got Message: ");
      // UART_msg_t_utils::print_msg(msg);
      //TODO: Implement MSG queue architecture

      return msg;
    }

    if (_recv_len < 0) 
    {
      //  logger::println("UARTHandler::poll->Packet requires shifting");
       // this only occurs if there was a timeout during the previous _recv_packet and an end flag before any new data

      //  logger::println("UARTHandler::poll->_msg_buffer after shifting: ");
      //  for (int i=0; i<(_recv_len + _partial_packet_len); i++)
      //  {
      //    logger::print(_msg_buffer[i], HEX); logger::print(", ");
      //  }
      //  logger::println();
       // append the _partial packet to the full message
       memcpy(_msg_buffer, _partial_packet, _partial_packet_len);
       
      //  logger::println("UARTHandler::poll->_msg_buffer after copyting _packed_data: ");
      //  for (int i=0; i<(_recv_len + _partial_packet_len); i++)
      //  {
      //    logger::print(_msg_buffer[i], HEX); logger::print(", ");
      //  }
      //  logger::println();
       _partial_packet_len = 0;
     }
    return empty_msg;
}

inline uint8_t UARTHandler::check_for_data()
{
    return MY_SERIAL.available();
}

void UARTHandler::_pack(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data, uint8_t *data_to_pack)
{
    // pack metadata
    data_to_pack[COMMAND] = msg_id;
    data_to_pack[JOINT_ID] = joint_id;
    
    // convert float array to short int array
    uint8_t _num_bytes = sizeof(float)/sizeof(UART_DATA_TYPE);
    uint8_t buf[_num_bytes];
    for (int i=0; i<len; i++)
    {
        //TODO: Check that the value isn't too large (328, -329), if it is warn the caller
        utils::float_to_short_fixed_point_bytes(data[i], buf, FIXED_POINT_FACTOR);
        uint8_t _offset = (DATA_START) + _num_bytes*i;
        memcpy((data_to_pack + _offset), buf, _num_bytes);
    }
}

UART_msg_t UARTHandler::_unpack(uint8_t* data, uint8_t len)
{
    //TODO: Check that len (argument) is even
    UART_msg_t msg;
    msg.command = data[COMMAND];
    msg.joint_id = data[JOINT_ID];
    float _total_len = len*sizeof(uint8_t);
    float _meta_len = sizeof(msg.command)+sizeof(msg.joint_id);
    float _conv_factor = ((float)sizeof(UART_DATA_TYPE)/(float)sizeof(float));
    msg.len = (_total_len - _meta_len) * _conv_factor;

    // fill msg.data, converting the short ints to floats
    for (int i=0; i<len; i++)
    {
        uint8_t _data_offset = (DATA_START) + (i*2);
        float tmp = 0;
        utils::short_fixed_point_bytes_to_float((uint8_t*)data+_data_offset, &tmp, FIXED_POINT_FACTOR);
        msg.data[i] = tmp;
    }
  
    return msg;
}

uint8_t UARTHandler::_get_packed_length(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data)
{
    uint8_t _val = 0;
    // we are converting from float to short int, we must multiply by the size difference
    _val += (float)len * (sizeof(float)/sizeof(UART_DATA_TYPE));
    _val += sizeof(msg_id);
    _val += sizeof(joint_id); 
    return _val;
}


void UARTHandler::_send_char(uint8_t val)
{
//  logger::print("UARTHandler::_send_char->Sending: 0x");
//  logger::println(val, HEX);
  MY_SERIAL.write(val);
}

uint8_t UARTHandler::_recv_char(void)
{  
  uint8_t _data = MY_SERIAL.read();
  // logger::print("UARTHandler::_recv_char->Read: "); logger::println(_data, HEX);

  return _data;
}

/* SEND_PACKET: sends a packet of length "len", starting at
  location "p".
*/
void UARTHandler::_send_packet(uint8_t* p, uint8_t len)
{
  /* send an initial END character to flush out any data that may
     have accumulated in the receiver due to line noise
  */
  _send_char(END);

  /* for each byte in the packet, send the appropriate character
     sequence
  */
  while (len--) {
    switch (*p) {
      /* if it's the same code as an END character, we send a
         special two character code so as not to make the
         receiver think we sent an END
      */
      case END:
        _send_char(ESC);
        _send_char(ESC_END);
        break;

      /* if it's the same code as an ESC character,
         we send a special two character code so as not
         to make the receiver think we sent an ESC
      */
      case ESC:
        _send_char(ESC);
        _send_char(ESC_ESC);
        break;

      /* otherwise, we just send the character
      */
      default:
        //logger::print("UARTHandler::_send_packet->Sending: 0x"); logger::println(*p, HEX);
        _send_char(*p);
    }

    p++;
  }

  /* tell the receiver that we're done sending the packet
  */
  _send_char(END);
}

/* RECV_PACKET: receives a packet into the buffer located at "p".
           If more than len bytes are received, the packet will
           be truncated.
           Returns the number of bytes stored in the buffer.
*/
int UARTHandler::_recv_packet(uint8_t *p, uint8_t len)
{
  uint8_t c;
  int received = 0;
  int bytes_left;
  
  _time_left(1);
  while (_time_left())
  {
//    logger::println("UARTHandler::_recv_packet->Loop");
    if (!check_for_data()) 
    {
      continue;
    }

    c = _recv_char();

//    logger::print("UARTHandler::_recv_packet->Got char: ");
//    logger::println(c);

    // handle bytestuffing if necessary
    switch (c) 
    {

      // if it's an END character then we're done with the packet
      case END:
        // logger::println("UARTHandler::_recv_packet->END CASE");
        if (received)
        {
            // logger::print("UARTHandler::_recv_packet->Returning: ");
            // logger::println(received);
            return received;
        }
        else if (_partial_packet_len)
        {
            //logger::print("UARTHandler::_recv_packet->Returning because of _partial_packet: ");
            return -1;
        }
        else
        {
            break;
        }

      /* if it's the same code as an ESC character, wait
         and get another character and then figure out
         what to store in the packet based on that.
      */
      case ESC:
//        logger::println("UARTHandler::_recv_packet->ESC CASE");
        c = _recv_char();
//        logger::print("UARTHandler::_recv_packet->ESC Char: ");
//        logger::println(c);

        /* if "c" is not one of these two, then we
           have a protocol violation.  The best bet
           seems to be to leave the byte alone and
           just stuff it into the packet
        */
        switch (c) 
        {
          case ESC_END:
//            logger::println("UARTHandler::_recv_packet->ESC_END");
            c = END;
            break;
          case ESC_ESC:
//            logger::println("UARTHandler::_recv_packet->ESC_ESC");
            c = ESC;
            break;
        }

      default:
//        logger::println("UARTHandler::_recv_packet->Default CASE");
        if (received < len)
        {
//          logger::println("UARTHandler::_recv_packet->Added to buffer");
          p[received++] = c;
        }
    }
  }

//  // there was a timeout before a full message was recieved, save the data to be reconstructed later
  //logger::println("UARTHandler::_recv_packet->Timeout!");
 _partial_packet_len = received;
//  logger::print("UARTHandler::_recv_packet->Saved Bytes: "); logger::println(_partial_packet_len);
 for (int i=0; i<_partial_packet_len; i++)
 {
   _partial_packet[i] = p[i];
  //  logger::print(_partial_packet[i], HEX); logger::println(", ");
 }
//  logger::println();

  return 0;
}


uint8_t UARTHandler::_time_left(uint8_t should_latch)
{
    static float _start_time;
    if (should_latch)
    {
      _start_time = micros();
//      logger::print("UARTHandler::_time_left->Latching on ");
//      logger::println(_start_time);
    }
    float del_t = micros() - _start_time;
//    logger::print("UARTHandler::_time_left->Del_T: ");
//    logger::println(micros() - _start_time);

    return (del_t <= _timeout_us);
}