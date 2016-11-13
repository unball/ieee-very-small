#include <elapsedMillis.h>

// Time calculations
elapsedMillis timeElapsed;

/**
 * Gets time interval since last reset.
 * @return Elapsed time in seconds.
 */
float getTimeInterval() {
  return timeElapsed/1000.0;
}

/**
 * Resets time counter.
 */
void resetTimer() {
  timeElapsed = 0;
}
