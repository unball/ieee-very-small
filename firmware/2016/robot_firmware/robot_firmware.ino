#include <control.h>
#include <IMU.h>
#include <radio.h>

#include <SPI.h>

bool has_received_message;

void setup() {
  Serial.begin(9600);
  radioSetup();
  motorsSetup();
  //IMUSetup();
}

void sendMessageBackToCentral() {
    int send_message[2];
    getMessages(send_message);
    send(send_message);   
}

void setSpeeds() {
  move(MOTOR_A, getMessage(0));
  move(MOTOR_B, getMessage(1));
  //setControlReference(MOTOR_A, getMessage(0));
  //setControlReference(MOTOR_B, getMessage(1));
}

void loop() {
  has_received_message = receive();
  Serial.println(getMessage(0));
  if (has_received_message){
      sendMessageBackToCentral();
      setSpeeds();
      Serial.println("aloha");
  }

  //controlMotors();
  
  delay(20);
  has_received_message = false;
}
