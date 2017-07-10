volatile long contadorA = 0;
volatile long contadorB = 0;
volatile long contador = 0;
long contadorA_media=0;
long contadorB_media=0;

void encodersSetup() {
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

void encoder() {
  interruptEncoderPins(Pins::channelA, contadorA);
  interruptEncoderPins(Pins::channelB, contadorB);

  contadorA_media+=(motorA_direction*contadorA-contadorA_media)/10;
  contadorB_media+=(motorB_direction*contadorB-contadorB_media)/10;
}