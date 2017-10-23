#define robot_number 3 //Define qual robô esta sendo configurado [1, 2, 3]
#include <pins.h>
#include <radio.h>
#include <motor.h>
#include <encoder.h>
#include <control.h>

void setup(void) {
  Serial.begin(115200);
  
  Radio::Setup();
  Motor::Setup();
  Encoder::Setup();
  Control::acc = 0;
}

void loop(){
  Control::stand();
}
