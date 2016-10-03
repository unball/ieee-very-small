#include <SPI.h>
#include "RF24.h"

int LED = 6;

/*Motor Variables*/
int PWMA = 9;
int AIN1 = 8;
int AIN2 = 7;
int PWMB = 5;
int BIN1 = 4;
int BIN2 = A3;

void setup() {
  radioSetup();
}

void loop() {
  bool has_received_message = receive();
  if (isStartingPipe()) {
    if (has_received_message)
      setChannel();
  }
  delay(10);
}
