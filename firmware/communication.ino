#include <UnBallSoftwareSerial.h>

// XBee communication (RX, TX)
UnBallSoftwareSerial serial1(15, 14);

float speedMap[8] = {-150, -100, -50, 0, 37.5, 75, 112.5, 150};

void startCommunication() {
  serial1.begin(9600);
}

/**
 * Receive message from XBee and decode.
 */
void receiveMessage() {
  unsigned char rcv_msg;
  int rcv_robot;
  int rcv_speed_left_code;
  int rcv_speed_right_code;
  
  if (serial1.available()) {
    rcv_msg = (unsigned char)serial1.read();
    Serial.print("Received: ");
    Serial.println(rcv_msg);
    decode(rcv_msg, &rcv_robot, &rcv_speed_left_code, &rcv_speed_right_code);

    if (rcv_robot == ROBOT_NUM) {
      speed_left = speedMap[rcv_speed_left_code];
      speed_right = speedMap[rcv_speed_right_code];
    }

    Serial.print("Speed left: ");
    Serial.print(speed_left);
    Serial.print(" Speed right: ");
    Serial.println(speed_right);
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
