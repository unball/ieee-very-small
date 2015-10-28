#include <elapsedMillis.h>

// Time calculations
elapsedMillis timeElapsed;

/**
 * Gets time interval since last reset.
 * @return Elapsed time in seconds.
 */
float getInterval() {
  return timeElapsed/1000.0;
}

/**
 * Prints time in serial port.
 */
void printTimer() {
  Serial.print("dt: ");
  Serial.println(getInterval());
}

/**
 * Resets time counter.
 */
void resetTimer() {
  timeElapsed = 0;
}
