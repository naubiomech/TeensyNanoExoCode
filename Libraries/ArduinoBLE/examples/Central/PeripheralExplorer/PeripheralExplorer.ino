/*
  Peripheral Explorer

  This example scans for BLE peripherals until one with a particular name ("LED")
  is found. Then connects, and discovers + prints all the peripheral's attributes.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use it with another board that is compatible with this library and the
  Peripherals -> LED example.

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

  logger::println("BLE Central - Peripheral Explorer");

  // start scanning for peripherals
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

    // see if peripheral is a LED
    if (peripheral.localName() == "LED") {
      // stop scanning
      BLE.stopScan();

      explorerPeripheral(peripheral);

      // peripheral disconnected, we are done
      while (1) {
        // do nothing
      }
    }
  }
}

void explorerPeripheral(BLEDevice peripheral) {
  // connect to the peripheral
  logger::println("Connecting ...");

  if (peripheral.connect()) {
    logger::println("Connected");
  } else {
    logger::println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  logger::println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    logger::println("Attributes discovered");
  } else {
    logger::println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // read and print device name of peripheral
  logger::println();
  logger::print("Device name: ");
  logger::println(peripheral.deviceName());
  logger::print("Appearance: 0x");
  logger::println(peripheral.appearance(), HEX);
  logger::println();

  // loop the services of the peripheral and explore each
  for (int i = 0; i < peripheral.serviceCount(); i++) {
    BLEService service = peripheral.service(i);

    exploreService(service);
  }

  logger::println();

  // we are done exploring, disconnect
  logger::println("Disconnecting ...");
  peripheral.disconnect();
  logger::println("Disconnected");
}

void exploreService(BLEService service) {
  // print the UUID of the service
  logger::print("Service ");
  logger::println(service.uuid());

  // loop the characteristics of the service and explore each
  for (int i = 0; i < service.characteristicCount(); i++) {
    BLECharacteristic characteristic = service.characteristic(i);

    exploreCharacteristic(characteristic);
  }
}

void exploreCharacteristic(BLECharacteristic characteristic) {
  // print the UUID and properties of the characteristic
  logger::print("\tCharacteristic ");
  logger::print(characteristic.uuid());
  logger::print(", properties 0x");
  logger::print(characteristic.properties(), HEX);

  // check if the characteristic is readable
  if (characteristic.canRead()) {
    // read the characteristic value
    characteristic.read();

    if (characteristic.valueLength() > 0) {
      // print out the value of the characteristic
      logger::print(", value 0x");
      printData(characteristic.value(), characteristic.valueLength());
    }
  }
  logger::println();

  // loop the descriptors of the characteristic and explore each
  for (int i = 0; i < characteristic.descriptorCount(); i++) {
    BLEDescriptor descriptor = characteristic.descriptor(i);

    exploreDescriptor(descriptor);
  }
}

void exploreDescriptor(BLEDescriptor descriptor) {
  // print the UUID of the descriptor
  logger::print("\t\tDescriptor ");
  logger::print(descriptor.uuid());

  // read the descriptor value
  descriptor.read();

  // print out the value of the descriptor
  logger::print(", value 0x");
  printData(descriptor.value(), descriptor.valueLength());

  logger::println();
}

void printData(const unsigned char data[], int length) {
  for (int i = 0; i < length; i++) {
    unsigned char b = data[i];

    if (b < 16) {
      logger::print("0");
    }

    logger::print(b, HEX);
  }
}
