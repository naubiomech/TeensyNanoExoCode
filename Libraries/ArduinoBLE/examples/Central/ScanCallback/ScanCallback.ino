/*
  Scan Callback

  This example scans for BLE peripherals and prints out their advertising details:
  address, local name, advertised service UUIDs. Unlike the Scan example, it uses
  the callback style APIs and disables filtering so the peripheral discovery is
  reported for every single advertisement it makes.

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

  logger::println("BLE Central scan callback");

  // set the discovered event handle
  BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);

  // start scanning for peripherals with duplicates
  BLE.scan(true);
}

void loop() {
  // poll the central for events
  BLE.poll();
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {
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
