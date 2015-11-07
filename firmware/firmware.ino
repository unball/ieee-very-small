// Robot identification number, from 0 to 2
#define ROBOT_NUM 0

// Alias for motors
#define MOTOR_A 1
#define MOTOR_B 0

// This time affects the amount of messages the robot can handle, as well
// as the precision of the control loop
const int LOOP_SLEEP_TIME = 10; // ms

// Motor driver standby
const int STBY = 6;

// Motor A
const int PWMA = 10; // Speed control 
const int AIN1 = 16; // Direction
const int AIN2 = 7;  // Direction
const int ENC_A_CH_A = 9; // Channel A
const int ENC_A_CH_B = 8; // Channel B
int speed_right = 0; // Motor A target speed

// Motor B
const int PWMB = 5;  // Speed control
const int BIN1 = A0; // Direction
const int BIN2 = A1; // Direction
const int ENC_B_CH_A = 3; // Channel A
const int ENC_B_CH_B = 4; // Channel B
int speed_left = 0; // Motor B target speed

void setup() {
  Serial.begin(115200);
  startCommunication();
  setPins();
  setEncoderInterrupts();
}

/**
 * Setup intput and output pins.
 */
void setPins() {
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(ENC_A_CH_A, INPUT);
  pinMode(ENC_A_CH_B, INPUT);
  pinMode(ENC_B_CH_A, INPUT);
  pinMode(ENC_B_CH_B, INPUT);
}

void loop() {
  receiveMessage();
  estimateSpeeds();
  controlMotors();

//  debugCommunication();
//  debugControl();
//  printSpeeds();

  resetEncoders();
  resetTimer();

  delay(LOOP_SLEEP_TIME);
}
