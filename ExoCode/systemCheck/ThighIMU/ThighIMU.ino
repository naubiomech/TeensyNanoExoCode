// Teensy code to connect to two thigh IMUs
#include "src/I2CHandler.h"
#include "src/Utilities.h"

#include "src/ThIMU.h"

ThIMU left = ThIMU(true);

void setup() {
  Serial.begin(115200);
  delay(100);
  while(!Serial);

  if (!left.init()) {
    logger::println("Failed Left Init");
  }
  
}

void loop() {
  //static I2C* i2c = I2C::get_instance();
  static uint8_t data;

//  logger::print("Reading from I2C device: ");
//  logger::print(i2c_cmds::thigh_imu::left_addr);
//  logger::print(" at register: ");
//  logger::print(i2c_cmds::thigh_imu::get_angle::reg);
//  logger::print(" with length: ");
//  logger::println(i2c_cmds::thigh_imu::get_angle::len);

  data = left.read_data();

  logger::print("Got: ");
  logger::println(data);
  
  
  // Try to read at 100Hz
  delay(100);
}
