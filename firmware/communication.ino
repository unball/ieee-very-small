#include <UnBallSoftwareSerial.h>

// XBee communication (RX, TX)
UnBallSoftwareSerial serial1(15, 14);

// Conversion from message speed to RPM
float speedMap[8] = {-150, -125, -100, 0, 75, 100, 125, 150};

// Flag to print communication data for debug purposes
bool debugCommunicationFlag = false;

/**
 * Set XBee to start communicating.
 */
void startCommunication() {
  serial1.begin(9600);
}

/**
 * Configure to print communication data for debug purposes.
 */
void debugCommunication() {
  debugCommunicationFlag = true;
}

/**
 * Receive message from XBee, decode, and save information to correspondent
 * global variables.
 */
void receiveMessage() {
  unsigned char rcv_msg;
  int rcv_robot;
  int rcv_speed_left_code;
  int rcv_speed_right_code;
  
  if (serial1.available()) {
    rcv_msg = (unsigned char)serial1.read();
    decode(rcv_msg, &rcv_robot, &rcv_speed_left_code, &rcv_speed_right_code);

    if (rcv_robot == ROBOT_NUM) {
      speed_left = speedMap[rcv_speed_left_code];
      speed_right = speedMap[rcv_speed_right_code];

      if (debugCommunicationFlag) {
        Serial.print("Received: ");
        Serial.println(rcv_msg);
        Serial.print("Received left: ");
        Serial.print(rcv_speed_left_code);
        Serial.print(" Received right: ");
        Serial.println(rcv_speed_right_code);
        Serial.print("Target speed left: ");
        Serial.print(speed_left);
        Serial.print(" Target speed right: ");
        Serial.println(speed_right);
      }
    }
  }
}

/**
 * Decode message according to the communication protocol.
 * @param msg: Received message.
 * @return robot: Robot number.
 * @return speed_left: Left motor speed.
 * @return speed_right: Right motor speed.
 */
void decode(char msg, int *robot, int *speed_left, int *speed_right) {
    *robot = (msg >> 6) & 0b11;
    *speed_left = (msg >> 3) & 0b111;
    *speed_right = msg & 0b111;
}
