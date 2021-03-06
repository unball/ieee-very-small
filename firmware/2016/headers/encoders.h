#include <pins.h>
#include <time.h>

volatile unsigned long contadorA = 0;
volatile unsigned long contadorB = 0;
volatile unsigned long contador = 0;

const int ENCODER_NUM_LINES = 512;
const int ENCODER_NUM_PULSES = ENCODER_NUM_LINES*19;

void encodersSetup() {
  pinMode(channelA, INPUT);  
  pinMode(channelB, INPUT);
}

void soma(){
  contador++;
}

void interruptEncoderPins(int channel, volatile unsigned long &contador_i) {
  contador = 0;
  attachInterrupt(channel, soma, RISING);
  delay(100);
  detachInterrupt(channel);
  contador_i = contador;  
}

void encoder() {
  interruptEncoderPins(channelA, contadorA);
  interruptEncoderPins(channelB, contadorB);
}

/**
 * Estimate speed based on encode pulses.
 * @param encoderPulses Encoder pulses since last loop.
 * @return Speed in RPM.
 */
int estimateSpeed(unsigned long encoderPulsesDiff) {
  //float dt = getTimeInterval();
  float dt = 0.1;
  return (int)(encoderPulsesDiff/(float)ENCODER_NUM_PULSES)*(60.0/dt);
}

void estimateSpeeds(int *speedA, int *speedB) {
  encoder();
  *speedA = contadorA;//estimateSpeed(contadorA);
  *speedB = contadorB;//estimateSpeed(contadorB);
}
