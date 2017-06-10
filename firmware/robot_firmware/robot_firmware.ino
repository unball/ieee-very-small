#include <pins.h>
#include <radio.h>
#include <motor.h>
#include <encoder.h>

long errorA_i=0;
long errorB_i=0;
long commandA_media=0;
long commandB_media=0;

int acc=0;

void setup(void) {

  Serial.begin(115200);
  
  radioSetup();
  motorsSetup();
  encodersSetup();
}

void loop(){
  stand();
}
