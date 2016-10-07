#include <SPI.h>
#include "RF24.h"

#define MOTOR_A "motorA"
#define MOTOR_B "motorB"

int LED = 6;
bool has_received_message;

void setup() {
  radioSetup();
  motorsSetup();
  IMUSetup();
}

void sendMessageBackToCentral() {
    int send_message[2];
    getMessages(send_message);
    send(send_message);   
}

void setSpeeds() {
  move(getMessage(0), MOTOR_A);
  setControlReference(MOTOR_A, getMessage(0));
  move(getMessage(1), MOTOR_B);
  setControlReference(MOTOR_B, getMessage(1));
}

void loop() {
  has_received_message = receive();

  if (has_received_message){
    if (isStartingPipe())
      setChannel();
    else {
      sendMessageBackToCentral();
      setSpeeds();
      controlMotors();
    }    
  }
  
  delay(20);
  has_received_message = false;
}
