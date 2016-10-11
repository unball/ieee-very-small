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
  pinMode(6, OUTPUT);
}

void sendMessageBackToCentral() {
    int send_message[2];
    getMessages(send_message);
    send(send_message);   
}

void getEncoderValues() {
    int send_message[2];
    send_message[0] = getMotorSpeed("motorA");
    send_message[1] = getMotorSpeed("motorB");
    send(send_message);  
}

void setSpeeds() {
  //move(getMessage(0), MOTOR_A);
  //move(getMessage(1), MOTOR_B);
  setControlReference(MOTOR_A, getMessage(0));
  setControlReference(MOTOR_B, getMessage(1));
}

void loop() {
  has_received_message = receive();
  digitalWrite(LED, LOW);

  if (has_received_message){
      sendMessageBackToCentral();
      digitalWrite(LED, HIGH);
      setSpeeds();
  }

  controlMotors();
  
  delay(20);
  has_received_message = false;
}
