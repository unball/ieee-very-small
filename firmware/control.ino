// Current motor speeds
float speedA = 0;
float speedB = 0;

// Motor PID control
const int MAX_MOTOR_POWER = 255;
const float MAX_CONTROL_OUTPUT = MAX_MOTOR_POWER;
const float KP = 1; // Best constant: 1
const float KI = 0; // Best constant: 0
const float KD = 0.1; // Best constant: 0.1
float errorAInt = 0;
float errorBInt = 0;
float errorAPrev = 0;
float errorBPrev = 0;
float prevTargetA = 0;
float prevTargetB = 0;

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
    power = control(speedA, targetSpeed, &errorAInt, &errorAPrev, &prevTargetA);
  } else {
    power = control(speedB, targetSpeed, &errorBInt, &errorBPrev, &prevTargetB);
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
 * @param prevTarget: Previous desired motor speed in RPM.
 * @return Motor output power.
 */
int control(float current, float target, float *errorInt, float *errorPrev,
  float *prevTarget) {
  int power;
  float dt;
  float errorP;
  float errorI;
  float errorD;
  
  // Somehow, the integral error initializes as NaN
  if (isnan(*errorInt))
    (*errorInt) = 0;

  // Reset control variables when changing targets
  if (*prevTarget != target) {
    *prevTarget = target;
    *errorInt = 0;
    *errorPrev = 0;
  }

  if (target == 0) {
    // Special case where the target is zero
    // Automatically stop and reset all error data
    (*errorInt) = 0;
    (*errorPrev) = 0;
    power = 0;
  } else {
    // PID errors
    dt = getTimeInterval();
    errorP = target - current;
    errorI = (*errorInt) + (errorP*dt);
    errorD = (errorP - (*errorPrev))/dt;
  
    // Limit integral error
    if (errorI > MAX_CONTROL_OUTPUT)
      errorI = MAX_CONTROL_OUTPUT;
    else if (errorI < -MAX_CONTROL_OUTPUT)
      errorI = -MAX_CONTROL_OUTPUT;
  
    // PID control law
    power = KP*errorP + KI*errorI + KD*errorD;

    // Limit power
    if (power > MAX_CONTROL_OUTPUT)
      power = MAX_CONTROL_OUTPUT;
    else if (power < -MAX_CONTROL_OUTPUT)
      power = -MAX_CONTROL_OUTPUT;
  
    // Store data for next loop
    *errorPrev = errorP;
    *errorInt = errorI;

    // Debug
    if (debugControlFlag) {
      Serial.print(" Power: ");
      Serial.print(power);
      Serial.print(" P: ");
      Serial.print(KP*errorP);
      Serial.print(" I: ");
      Serial.print(KI*errorI);
      Serial.print(" D: ");
      Serial.print(KD*errorD);
      Serial.println();
    }
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
