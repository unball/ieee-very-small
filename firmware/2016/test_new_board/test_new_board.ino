#include <SPI.h>
#include "RF24.h"

#define MOTOR_A "motorA"
#define MOTOR_B "motorB"

int motor_power = 0;

int LED = 6;

void setup() {
  radioSetup();
  motorsSetup();
}

void testCommunicationWithCentral() {
  bool has_received_message = receive();
  if (has_received_message) {
    int send_message[2];
    getMessages(send_message);
    send(send_message);      
  }
}

void testRadio() {
  checkIfRadioIsPVariant();
  testCommunicationWithCentral();
}


int sign = 1;
void defineDirection() {
  if (abs(motor_power) > 100)
    sign = -sign;
}

void testMotors() {
  defineDirection();
  motor_power += sign;
  
  move(motor_power, MOTOR_A);
  move(motor_power, MOTOR_B);  
}

void loop() {
  testRadio();
  testMotors();
  
  delay(10);
}
