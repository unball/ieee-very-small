#include <SPI.h>
#include "RF24.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI protocol*/
RF24 radio(3, 2);   //arduino nano
//RF24 radio(10,A0); //arduino pro micro
/**********************************************************/

byte pipe[][6] = {"1Node", "2Node", "3Node", "4Node"};
int msg_from_robot[2]; //pipe number
int msg_from_ROS[3];

void setup() {
  Serial.begin(250000);
  Serial.println("Inicio - central");
  radio.begin();

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  radio.openReadingPipe(0, pipe[0]); //n1->central pelo pipe0
  radio.startListening();
}

void checkIfRadioIsPVariant() {
  if (radio.isPVariant()) {
    Serial.println("versao nrf24l01+");
  }
}

bool receive_data_from_robot() {
  if (radio.available()) {
    while (radio.available()) {
      radio.read(&msg_from_robot, sizeof(msg_from_robot));
    }
    return true;
  }
  return false;
}

void send_data_to_ROS() {
  Serial.println(msg_from_robot[0]); //Serial.write() may be a better choice for this
  Serial.println(msg_from_robot[1]); //Serial.write() may be a better choice for this
}

bool receive_data_from_ROS() {
  if (Serial.available()) {
    int i = 0;
    while (Serial.available()) {
      msg_from_ROS[i++] = Serial.read() - 48;
    }
    parseROStoRobot();
    return true;
  }
  return false;
}

int commands[2];
int robotPipe = 1;

void parseROStoRobot() {
  robotPipe = msg_from_ROS[0];
  commands[0] = msg_from_ROS[1];
  commands[1] = msg_from_ROS[2];
}

void send_data_to_robot() {
  radio.stopListening();
  radio.openWritingPipe(pipe[robotPipe]);
  radio.write(commands, sizeof(commands));
  radio.startListening();
}

void loop() {
  if (receive_data_from_robot())
    send_data_to_ROS();
  if (receive_data_from_ROS())
    send_data_to_robot();

  delay(10);
}

