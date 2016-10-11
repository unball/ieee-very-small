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
      resetTimer();
      sendMessageBackToCentral();
      digitalWrite(LED, HIGH);
      setSpeeds();
  }

  controlMotors();

  if (getMessage(0) != 0 && getMessage(1) != 0) {
    if (getTimeInterval() > 0.8) {
      int sign = random(0,3);
      if (sign == 0)
        sign = -1;
      else
        sign = 1;
      setControlReference(MOTOR_A, sign*(random(0.5,1.5)*getMessage(0) + random(20,80)));
      setControlReference(MOTOR_B, sign*(random(0.5,1.5)*getMessage(1) + random(20,80)));
      resetTimer();      
    }      
  }
  
  
  delay(20);
  has_received_message = false;
}
