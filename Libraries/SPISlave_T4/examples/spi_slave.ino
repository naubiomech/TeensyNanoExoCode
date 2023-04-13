#include "SPISlave_T4.h"
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  mySPI.begin();
  //    mySPI.onReceive(myFunc);
}

void loop() {
  logger::print("millis: "); logger::println(millis());
  delay(1000);
}

void myFunc() {
  logger::println("START: ");
  uint8_t arr[] = { 3, 2, 8, 3, 10, 11, 33, 13, 14 };
  uint8_t i = 0;
  while ( mySPI.active() ) {
    if (mySPI.available()) {
      if ( i++ > sizeof(arr) ) i = 0;
      mySPI.pushr(arr[i]);
      logger::print("VALUE: ");
      logger::println(mySPI.popr());
    }
  }
  logger::println("END");
}
