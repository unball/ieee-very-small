#include <pins.h>
#include <motors.h>
#include <encoders.h>

// Current motor speeds in RPM
int speedA = 0;
int speedB = 0;

int reference_speedA;
int reference_speedB;

// Motor PID control
const int MAX_MOTOR_POWER = 255;
const float MAX_CONTROL_OUTPUT = 255;
const float KP = 2;
const float KI = 5;
const float KD = 0.01;
float errorA_int = 0;
float errorA_prev = 0;
float prev_targetA = 0;

float errorB_int = 0;
float errorB_prev = 0;
float prev_targetB = 0;


void setControlReference(String motor, int reference_speed) {
  if (motor == MOTOR_A)
    reference_speedA = reference_speed;
  if (motor == MOTOR_B)
    reference_speedB = reference_speed;
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
  }

  return power;
}

/**
 * Control specified motor to reach target speed.
 * @param motor: Controlled motor.
 * @param target_speed: Desired speed in RPM.
 */
void controlMotor(String motor, float target_speed) {
  int power;
  
  if (motor == MOTOR_A) {
    power = control(speedA, target_speed, &errorA_int, &errorA_prev, &prev_targetA);
  } else {
    power = control(speedB, target_speed, &errorB_int, &errorB_prev, &prev_targetB);
  }

  move(motor, power);
}

/**
 * Control motors with current target speed.
 */
void controlMotors() {
  estimateSpeeds(&speedA, &speedB);
  float percent_errorA = (reference_speedA - speedA)/reference_speedA;
  float percent_errorB = (reference_speedB - speedB)/reference_speedB;
  float errors_difference = percent_errorA - percent_errorB;
  reference_speedA *= (1 + errors_difference);
  reference_speedB *= (1 - errors_difference);
  controlMotor(MOTOR_A, reference_speedA);
  controlMotor(MOTOR_B, reference_speedB);  
}