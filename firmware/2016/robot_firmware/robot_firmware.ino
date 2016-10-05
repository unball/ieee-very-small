#include <SPI.h>
#include "RF24.h"

int LED = 6;

void setup() {
  Serial.begin(250000);
  while(!Serial);
    Serial.println("Inicio - robo");
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
}
