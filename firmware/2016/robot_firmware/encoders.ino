int channelA = 3; //TX
int channelB = 2; //RX

volatile unsigned long contadorA = 0;
volatile unsigned long contadorB = 0;
volatile unsigned long contador = 0;

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
