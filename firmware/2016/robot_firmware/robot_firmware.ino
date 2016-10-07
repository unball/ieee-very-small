#include <SPI.h>
#include "RF24.h"

int LED = 6;
bool has_received_message;

void setup() {
  radioSetup();
  motorsSetup();
  IMUSetup();
}

void verifyPipe() {
  if (isStartingPipe()) {
    if (has_received_message)
      setChannel();
  }  
}

void loop() {
  has_received_message = receive();
  verifyPipe();

  int send_message[2];
  getMessages(send_message);
  send(send_message);
  
  delay(20);

  move (100, "motorA");
  move (-100,"motorB");

  has_received_message = false;
}
