/*
  Scan

  This example scans for BLE peripherals and prints out their advertising details:
  address, local name, advertised service UUID's.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    logger::println("starting BLE failed!");

    while (1);
  }

  logger::println("BLE Central scan");

  // start scanning for peripheral
  BLE.scan();
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral
    logger::println("Discovered a peripheral");
    logger::println("-----------------------");

    // print address
    logger::print("Address: ");
    logger::println(peripheral.address());

    // print the local name, if present
    if (peripheral.hasLocalName()) {
      logger::print("Local Name: ");
      logger::println(peripheral.localName());
    }

    // print the advertised service UUIDs, if present
    if (peripheral.hasAdvertisedServiceUuid()) {
      logger::print("Service UUIDs: ");
      for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
        logger::print(peripheral.advertisedServiceUuid(i));
        logger::print(" ");
      }
      logger::println();
    }

    // print the RSSI
    logger::print("RSSI: ");
    logger::println(peripheral.rssi());

    logger::println();
  }
}
