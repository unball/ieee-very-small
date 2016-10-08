int channelA = 3; //TX
int channelB = 2; //RX

unsigned long contadorA = 0;
unsigned long contadorB = 0;
volatile unsigned long contador = 0;

const int ENCODER_NUM_LINES = 512;
const int ENCODER_NUM_PULSES = ENCODER_NUM_LINES*19;
#define INTERRUPT_DURATION 100

void encodersSetup() {
  pinMode(channelA, INPUT);  
  pinMode(channelB, INPUT);
}

void estimateSpeeds(float *speedA, float *speedB) {
  encoder();
  //int contadores[2] = {contadorA, contadorB};
  //send(contadores);
  int speeds[2];
  speeds[0] = estimateSpeed(contadorA);
  speeds[1] = estimateSpeed(contadorB);
  send(speeds);
  //*speedA = estimateSpeed(contadorA);
  //*speedB = estimateSpeed(contadorB);
}

/**
 * Estimate speed based on encode pulses.
 * @param encoderPulses Encoder pulses since last loop.
 * @return Speed in RPM.
 */
float estimateSpeed(unsigned long encoderPulsesDiff) {
  float rotations_per_interruption = (float)encoderPulsesDiff/(float)ENCODER_NUM_PULSES;
  float rotations_per_second = rotations_per_interruption*(1000/INTERRUPT_DURATION);
  float rotations_per_minute = rotations_per_second*60;
  return (rotations_per_minute);
}

void interruptEncoderPins(int channel, volatile unsigned long &contador_i) {
  contador = 0;
  attachInterrupt(channel, soma, RISING);
  delay(INTERRUPT_DURATION);
  detachInterrupt(channel);
  contador_i = contador;  
}

void soma(){
  contador++;
}

void encoder() {
  interruptEncoderPins(channelA, contadorA);
  interruptEncoderPins(channelB, contadorB);
}
