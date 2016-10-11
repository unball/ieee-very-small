#include <SPI.h>
#include "RF24.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI protocol*/
//RF24 radio(3, 2);   //arduino nano
RF24 radio(10, A0); //arduino pro micro
/**********************************************************/

byte pipe[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node", "7Node", "8Node"};
int msg_from_robot[2];
int msg_from_ROS[3];

int commands[2] = {0, 0};
int robotPipe;

bool robot_online = false;

void setup() {
  Serial.begin(19200);
  //  while(!Serial)
  //    Serial.println("Inicio - central");
  radio.begin();

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  radio.openReadingPipe(1, pipe[4]); //n1->central pelo pipe0
  //radio.openReadingPipe(2, pipe[4]); //n1->central pelo pipe0
  //radio.openReadingPipe(3, pipe[6]); //n1->central pelo pipe0
  //radio.openReadingPipe(4, pipe[7]); //n1->central pelo pipe0
  radio.startListening();
}

void checkIfRadioIsPVariant() {
  if (radio.isPVariant()) {
    Serial.println("versao nrf24l01+");
  }
}

bool receiveDataFromAnotherRadio() {
  if (radio.available()) {
    while (radio.available()) {
      radio.read(&msg_from_robot, sizeof(msg_from_robot));
    }
    return true;
  }
  return false;
}

void sendDataToSerialPort() {
  Serial.println(msg_from_robot[0]); //Serial.write() may be a better choice for this
  Serial.println(msg_from_robot[1]); //Serial.write() may be a better choice for this
}

bool receiveDataFromSerialPort() {
  if (Serial.available()) {
    int i = 0;
    while (i < 3) {
      msg_from_ROS[i++] = Serial.read();
    }
    parseSerialMessageToRobot();
    return true;
  }
  return false;
}

void parseSerialMessageToRobot() {
  robotPipe = msg_from_ROS[0];
  commands[0] = msg_from_ROS[1];
  commands[1] = msg_from_ROS[2];
}

void sendDataToRobot() {
  radio.stopListening();
  radio.openWritingPipe(pipe[robotPipe]);
  robot_online = radio.write(commands, sizeof(commands));
  //Serial.println(robot_online);
  radio.startListening();
}

void loop() {
  if (receiveDataFromAnotherRadio())
    sendDataToSerialPort();
  if (receiveDataFromSerialPort())
    sendDataToRobot();

  delay(10);
}

