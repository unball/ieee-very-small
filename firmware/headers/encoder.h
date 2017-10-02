#ifndef ENCODER_H
#define ENCODER_H
#include <pins.h>
#include <motor.h>

namespace Encoder {
  volatile long contadorA = 0;
  volatile long contadorB = 0;
  volatile long contador = 0;
  long contadorA_media = 0;
  long contadorB_media = 0;

  void Setup() {
    pinMode(Pins::channelA, INPUT);  
    pinMode(Pins::channelB, INPUT);
  }

  void soma(){
    contador++;
  }

  void interruptEncoderPins(int channel, volatile long &contador_i) {
    contador = 0;
    attachInterrupt(channel, soma, RISING);
    delay(10);
    detachInterrupt(channel);
    contador_i = contador;  
  }

  void resetEncoders() {
      contadorA_media=0;
      contadorB_media=0;      
  }

  void encoder() {
    interruptEncoderPins(Pins::channelA, contadorA);
    interruptEncoderPins(Pins::channelB, contadorB);

    float a = 0.9; // SEMPRE 0 <= a <= 1
    contadorA_media = a*contadorA_media + (1 - a)*Motor::motorA_direction*contadorA;
    contadorB_media = a*contadorB_media + (1 - a)*Motor::motorB_direction*contadorB;
    /*Serial.print("\ncontadorA: ");
    Serial.print(contadorA);
    Serial.print("\t contadorB: ");
    Serial.println(contadorB);
    Serial.print("\n");*/
  }
}

#endif