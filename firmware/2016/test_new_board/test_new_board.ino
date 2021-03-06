#include <control.h>
#include <radio.h>

#include <SPI.h>

int motor_power = 0;

int LED = 6;

void setup() {
  radioSetup();
  motorsSetup();
  encodersSetup();
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
  if (abs(motor_power) > 50)
    sign = -sign;
}

void testMotors() {
  motor_power += sign;
  Serial.println(motor_power);
  defineDirection();
  
  move(MOTOR_A, motor_power);
  move(MOTOR_B, motor_power);  
}

void testEncoders() {
  int speed1, speed2;
  estimateSpeeds(&speed1, &speed2);
  Serial.print("SpeedA = "); Serial.print(speed1);
  Serial.print(", SpeedB = "); Serial.println(speed2); 
}

void loop() {
  //testRadio();
  testMotors();
  testEncoders();
  
  delay(100);
}
