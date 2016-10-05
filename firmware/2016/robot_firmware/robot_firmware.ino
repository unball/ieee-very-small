#include <SPI.h>
#include "RF24.h"

int LED = 6;

void setup() {
  radioSetup();
  motorsSetup();
  IMUSetup();
}

void loop() {
  bool has_received_message = receive();
  if (isStartingPipe()) {
    if (has_received_message)
      setChannel();
  }
  delay(20);

  move(100, "motorA");
  move(-100, "motorB");
}
