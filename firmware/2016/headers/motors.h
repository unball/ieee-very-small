#include <pins.h>

#define MOTOR_A "motorA"
#define MOTOR_B "motorB"

void setMotorPin(int PWM, int IN1, int IN2) {  
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);  

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void motorsSetup() {
  setMotorPin(PWMA, AIN1, AIN2);  
  setMotorPin(PWMB, BIN1, BIN2);
  digitalWrite(STDBY, HIGH);
}

void move(int power, int direction, int PWM, int IN1, int IN2) {
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  digitalWrite(IN1, inPin1);
  digitalWrite(IN2, inPin2);
  analogWrite(PWM, power);
}

/*motor <- "motorA" or "motorB"*/
void move(String motor, int power) {
  int pin1, pin2;
  int PWM;
  if (motor == "motorA") {
    pin1 = AIN1;
    pin2 = AIN2;
    PWM = PWMA;
  }
  else {
    pin1 = BIN1;
    pin2 = BIN2;
    PWM = PWMB;    
  }
  
  power = map(power,-100,100,-255,255);
  int power_magnitude = abs(power);
  int motor_direction = (power > 0 ? 1:0);
  move(power_magnitude, motor_direction, PWM, pin1, pin2);
}