/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* Set the delay between fresh samples (not too fast, BLE UART is slow!) */
/* Firware <=0.6.6 should use 500ms, >=0.6.7 can use 200ms */
#define BNO055_SAMPLERATE_DELAY_MS (500)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*!
    @brief  A small helper function for error messages
*/
/**************************************************************************/
void error(const __FlashStringHelper*err)
{
  logger::println(err);
  while (1);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  logger::println("------------------------------------");
  logger::print  ("Sensor:       "); logger::println(sensor.name);
  logger::print  ("Driver Ver:   "); logger::println(sensor.version);
  logger::print  ("Unique ID:    "); logger::println(sensor.sensor_id);
  logger::print  ("Max Value:    "); logger::print(sensor.max_value); logger::println(" xxx");
  logger::print  ("Min Value:    "); logger::print(sensor.min_value); logger::println(" xxx");
  logger::print  ("Resolution:   "); logger::print(sensor.resolution); logger::println(" xxx");
  logger::println("------------------------------------");
  logger::println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  // Get the system status values (mostly for debugging purposes)
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  // Display the results in the Serial Monitor
  logger::print("System Status: 0x");
  logger::println(system_status, HEX);
  logger::print("Self Test:     0x");
  logger::println(self_test_results, HEX);
  logger::print("System Error:  0x");
  logger::println(system_error, HEX);
  logger::println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  // Get the four calibration values (0..3)
  // 3 means 'fully calibrated"
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // The data should be ignored until the system calibration is > 0
  logger::print("\t");
  if (!system)
  {
    logger::print("! ");
  }

  // Display the individual values
  logger::print("Sys:");
  logger::print(system, DEC);
  logger::print(" G:");
  logger::print(gyro, DEC);
  logger::print(" A:");
  logger::print(accel, DEC);
  logger::print(" M:");
  logger::print(mag, DEC);
}

/**************************************************************************/
/*
    Sends sensor calibration status out over BLE UART
*/
/**************************************************************************/
void transmitCalStatus(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  // Get the four calibration values (0..3)
  // 3 means 'fully calibrated"
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* Prepare the AT command */
  ble.print("AT+BLEUARTTX=");

  // Transmit individual values
  // Note: The values are abbreviated compared to the Serial Monitor
  // to save space since BLE UART is quite slow */
  ble.print(",");
  ble.print(system, DEC);
  ble.print(gyro, DEC);
  ble.print(accel, DEC);
  ble.println(mag, DEC);

  if (! ble.waitForOK() )
  {
    logger::println(F("Failed to send?"));
  }
}

/**************************************************************************/
/*!
    @brief  Initializes the one wire temperature sensor
*/
/**************************************************************************/
void initSensor(void)
{
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    logger::print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  // Display some basic information on this sensor
  displaySensorDetails();

  // Optional: Display current status
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  logger::println(F("Adafruit Bluefruit Command Mode Example"));
  logger::println(F("---------------------------------------"));

  // Initialise the module
  logger::print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  logger::println( F("OK!") );

  // Perform a factory reset to make sure everything is in a known state
  logger::println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  logger::println("Requesting Bluefruit info:");
  // Print Bluefruit information
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  // Setup the BNO055 sensor
  initSensor();

  logger::println("Waiting for a BLE connection to continue ...");

  // Wait for connection to finish
  while (! ble.isConnected()) {
      delay(5000);
  }

  logger::println(F("CONNECTED!"));
  logger::println(F("**********"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for user input
  char inputs[BUFSIZE+1];

  // Send sensor data out
  if (ble.isConnected())
  {
    // Get Quaternion data (no 'Gimbal Lock' like with Euler angles)
    imu::Quaternion quat = bno.getQuat();

    // Display the full data in Serial Monitor
    logger::print("qW: ");
    logger::print(quat.w(), 4);
    logger::print(" qX: ");
    logger::print(quat.y(), 4);
    logger::print(" qY: ");
    logger::print(quat.x(), 4);
    logger::print(" qZ: ");
    logger::print(quat.z(), 4);
    displayCalStatus();
    logger::println("");

    // Send abbreviated integer data out over BLE UART
    ble.print("AT+BLEUARTTX=");
    ble.print(quat.w(), 4);
    ble.print(",");
    ble.print(quat.y(), 4);
    ble.print(",");
    ble.print(quat.x(), 4);
    ble.print(",");
    ble.println(quat.z(), 4);
    if (! ble.waitForOK() )
    {
      logger::println(F("Failed to send?"));
    }

    // Optional: Send the calibration data as well
    transmitCalStatus();

    // Send a new line character for the next record
    ble.println("AT+BLEUARTTX=\\r\\n");
    if (! ble.waitForOK() )
    {
      logger::println(F("Failed to send?"));
    }

    /*
    // Display the buffer size (firmware 0.6.7 and higher only!)
    ble.println("AT+BLEUARTFIFO=TX");
    ble.readline();
    logger::print("TX FIFO: ");
    logger::println(ble.buffer);
    */

    // Wait a bit ...
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }

  // Check for incoming characters from Bluefruit
  if (ble.isConnected())
  {
    ble.println("AT+BLEUARTRX");
    ble.readline();
    if (strcmp(ble.buffer, "OK") == 0) {
      // no data
      return;
    }
    // Some data was found, its in the buffer
    logger::print(F("[Recv] ")); logger::println(ble.buffer);
    ble.waitForOK();
  }
}
