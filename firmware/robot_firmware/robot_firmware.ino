#define robot_number 1 //Define qual robô esta sendo configurado [0->placa 2] [1->placa 3] [2->placa 6]
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
