#include <control.h>
#include <IMU.h>
#include <radio.h>

#include <SPI.h>

int LED = 6;
bool has_received_message;

void setup() {
  radioSetup();
  motorsSetup();
  IMUSetup();
  pinMode(LED, OUTPUT);
}

void sendMessageBackToCentral() {
    int send_message[2];
    getMessages(send_message);
    send(send_message);   
}

void setSpeeds() {
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
