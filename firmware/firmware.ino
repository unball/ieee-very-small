#define ROBOT_NUM 0
#define MOTOR_A 1
#define MOTOR_B 0

// Motor driver standby
const int STBY = 6;

// Motor A
const int PWMA = 10; // Speed control 
const int AIN1 = 16; // Direction
const int AIN2 = 7;  // Direction
const int ENC_A_CH_A = 9; // Channel A
const int ENC_A_CH_B = 8; // Channel B
int speed_right = 0; // Motor A speed

// Motor B
const int PWMB = 5;  // Speed control
const int BIN1 = A0; // Direction
const int BIN2 = A1; // Direction
const int ENC_B_CH_A = 3; // Channel A
const int ENC_B_CH_B = 4; // Channel B
int speed_left = 0; // Motor B speed

void setup() {
  Serial.begin(9600);
  startCommunication();
  setPins();
  setEncoderInterrupts();
}

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
  calculateSpeeds();
  controlMotors();

  debugControl();
  printSpeeds();

  resetEncoders();
  resetTimer();
  delay(50);
}
