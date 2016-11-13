#include <motors.h>
#include <radio.h>

#include <SPI.h>

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
  
  move(MOTOR_A, motor_power);
  move(MOTOR_B, motor_power);  
}

void loop() {
  testRadio();
  testMotors();
  
  delay(10);
}
