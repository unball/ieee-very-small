#include <SPI.h>
#include "RF24.h"

#define MOTOR_A "motorA"
#define MOTOR_B "motorB"

int LED = 6;
bool has_received_message;

void setup() {
  radioSetup();
  motorsSetup();
}

void sendMessageBackToCentral() {
    int send_message[2];
    getMessages(send_message);
    send(send_message);   
}

void setSpeeds() {
  move(100, MOTOR_A);
  move(100, MOTOR_B);
}

void loop() {
  has_received_message = receive();

  setSpeeds();
   
  if (has_received_message){
    if (isStartingPipe())
      setChannel();
  }
  
  delay(20);
  has_received_message = false;
}
