// Current motor speeds
float speedA = 0;
float speedB = 0;

// Motor PID control
const int MAX_MOTOR_POWER = 255;
const float MAX_ERROR_I = 1000;
const float KP = 0.2;
const float KI = 0.1;
const float KD = 0;
float errorAInt = 0;
float errorBInt = 0;
float errorAPrev = 0;
float errorBPrev = 0;

// Flag to print control data for debug purposes
bool debugControlFlag = false;

/**
 * Get current motor speeds.
 * @return Current motor speed in RPM.
 */
float getMotorSpeed(int motor) {
  if (motor == MOTOR_A)
    return speedA;
  return speedB;
}

/**
 * Control motors with current target speed.
 */
void controlMotors() {
  controlMotor(MOTOR_A, speed_right);
  controlMotor(MOTOR_B, speed_left);
}

/**
 * Control specified motor to reach target speed.
 * @param motor: Controlled motor.
 * @param targetSpeed: Desired speed in RPM.
 */
void controlMotor(int motor, float targetSpeed) {
  int power;
  
  if (motor == MOTOR_A) {
    power = control(speedA, targetSpeed, &errorAInt, &errorAPrev);
  } else {
    power = control(speedB, targetSpeed, &errorBInt, &errorBPrev);
  }

  setMotorPower(motor, power);
}

/**
 * Configure to print control data for debug.
 */
void debugControl() {
  debugControlFlag = true;
}

/**
 * Motor control law.
 * @param current: Current motor speed in RPM.
 * @param target: Desired motor speed in RPM.
 * @param errorInt: Accumulated speed error.
 * @param errorPrev: Error from the last iteration.
 * @return Motor output power.
 */
int control(float current, float target, float *errorInt, float *errorPrev) {
  // Somehow, the integral error initializes as NaN
  if (isnan(*errorInt))
    (*errorInt) = 0;

  // PID errors
  float errorP = target - current;
  float errorI = (*errorInt) + errorP;
  float errorD = (*errorPrev) - target;

  // Limit integral error
  errorI = errorI > MAX_ERROR_I ? MAX_ERROR_I : errorI;

  // PID control law
  int power = KP*errorP + KI*errorI + KD*errorD;

  // Store data for next loop
  *errorPrev = errorP;
  *errorInt = errorI;

  // Debug
  if (debugControlFlag) {
    Serial.print(" Power: ");
    Serial.print(power);
    Serial.print(" P: ");
    Serial.print(errorP);
    Serial.print(" I: ");
    Serial.print(errorI);
    Serial.print(" D: ");
    Serial.print(errorD);
    Serial.println();
  }

  return power;
}

/**
 * Set output power for a motor using the PWM pin.
 * @param motor: Controlled motor.
 * @param power: Output power, which 0 is off, 255 is full power clockwise
 *   and -255 is full power counterclockwise.
 */
void setMotorPower(int motor, int power) {
  boolean inPin1;
  boolean inPin2;
  
  enableMotors();

  if (power > 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  } else {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (abs(power) > MAX_MOTOR_POWER)
    power = MAX_MOTOR_POWER;
  else
    power = abs(power);

  if (motor == MOTOR_B) {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, abs(power));
  } else {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, abs(power));
  }
}

/**
 * Enable motors to run.
 */
void enableMotors() {
  digitalWrite(STBY, HIGH);
}

/**
 * Disable motors to run.
 */
void disableMotors() {
  digitalWrite(STBY, LOW);
}

/**
 * Stop all motors.
 */
void stop() {  
  disableMotors();
}
