#include <SPI.h>
#include "RF24.h"

int LED = 6;

void setup() {
  radioSetup();
}

void loop() {
  bool has_received_message = receive();
  if (isStartingPipe()) {
    if (has_received_message)
      setChannel();
  }
  delay(10);
  move(25, "motorA");
  move(75, "motorB");
}
