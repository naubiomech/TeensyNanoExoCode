/*
  SensorTag Button

  This example scans for BLE peripherals until a TI SensorTag is discovered.
  It then connects to it, discovers the attributes of the 0xffe0 service,
  subscribes to the Simple Key Characteristic (UUID 0xffe1). When a button is
  pressed on the SensorTag a notification is received and the button state is
  outputted to the Serial Monitor when one is pressed.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.
  - TI SensorTag

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

  logger::println("BLE Central - SensorTag button");
  logger::println("Make sure to turn on the device.");

  // start scanning for peripheral
  BLE.scan();
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    logger::print("Found ");
    logger::print(peripheral.address());
    logger::print(" '");
    logger::print(peripheral.localName());
    logger::print("' ");
    logger::print(peripheral.advertisedServiceUuid());
    logger::println();

    // Check if the peripheral is a SensorTag, the local name will be:
    // "CC2650 SensorTag"
    if (peripheral.localName() == "CC2650 SensorTag") {
      // stop scanning
      BLE.stopScan();

      monitorSensorTagButtons(peripheral);

      // peripheral disconnected, start scanning again
      BLE.scan();
    }
  }
}

void monitorSensorTagButtons(BLEDevice peripheral) {
  // connect to the peripheral
  logger::println("Connecting ...");
  if (peripheral.connect()) {
    logger::println("Connected");
  } else {
    logger::println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  logger::println("Discovering service 0xffe0 ...");
  if (peripheral.discoverService("ffe0")) {
    logger::println("Service discovered");
  } else {
    logger::println("Attribute discovery failed.");
    peripheral.disconnect();

    while (1);
    return;
  }

  // retrieve the simple key characteristic
  BLECharacteristic simpleKeyCharacteristic = peripheral.characteristic("ffe1");

  // subscribe to the simple key characteristic
  logger::println("Subscribing to simple key characteristic ...");
  if (!simpleKeyCharacteristic) {
    logger::println("no simple key characteristic found!");
    peripheral.disconnect();
    return;
  } else if (!simpleKeyCharacteristic.canSubscribe()) {
    logger::println("simple key characteristic is not subscribable!");
    peripheral.disconnect();
    return;
  } else if (!simpleKeyCharacteristic.subscribe()) {
    logger::println("subscription failed!");
    peripheral.disconnect();
    return;
  } else {
    logger::println("Subscribed");
    logger::println("Press the right and left buttons on your SensorTag.");
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

    // check if the value of the simple key characteristic has been updated
    if (simpleKeyCharacteristic.valueUpdated()) {
      // yes, get the value, characteristic is 1 byte so use byte value
      byte value = 0;
      
      simpleKeyCharacteristic.readValue(value);

      if (value & 0x01) {
        // first bit corresponds to the right button
        logger::println("Right button pressed");
      }

      if (value & 0x02) {
        // second bit corresponds to the left button
        logger::println("Left button pressed");
      }
    }
  }

  logger::println("SensorTag disconnected!");
}
