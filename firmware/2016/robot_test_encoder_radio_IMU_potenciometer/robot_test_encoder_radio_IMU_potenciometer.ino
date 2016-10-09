/*
Code for Receiver
http://www.instructables.com/id/Wireless-Remote-Using-24-Ghz-NRF24L01-Simple-Tutor
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
int ax, ay, az;
int gx, gy, gz;

bool sending = true;
int msg[2];

int PWMA=9;
int AIN1=8;
int AIN2=7;

int PWMB = 5;
int BIN1 = 4;
int BIN2 = A3;

int STBY=6;
int POT=A2;

int channelA = 3; //TX
int channelB = 2; //RX

volatile unsigned long prevContadorA = 0;
volatile unsigned long contadorA = 0;
volatile unsigned long prevContadorB = 0;
volatile unsigned long contadorB = 0;
volatile unsigned long contador = 0;

RF24 radio(10,A0);  //CE, CS

const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void){
    interrupts();
 
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);

    if (!sending) {
      radio.openReadingPipe(0,pipe);
      radio.startListening();
    }
    else {
      radio.openWritingPipe(pipe);  
    }
    
    pinMode(STBY, OUTPUT);
    pinMode(channelA, INPUT);
    pinMode(channelA, INPUT);  

    setMotorPin(PWMA, AIN1, AIN2);
    setMotorPin(PWMB, BIN1, BIN2);
    
    digitalWrite(STBY, HIGH);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif    
    
    accelgyro.initialize();
    accelgyro.testConnection();
}

void setMotorPin(int PWM, int IN1, int IN2) {  
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);  

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void printSpeedEncoder(volatile unsigned long prevContador, volatile unsigned long contador, String message ){
    float rpm = ((float)contador*100*60)/(512*19);
}

void interruptEncoderPins(int channel, volatile unsigned long &contador_i) {
  contador = 0;
  attachInterrupt(channel, soma, RISING);
  delay(10);
  detachInterrupt(channel);
  contador_i = contador;  
}

void encoder() {
  interruptEncoderPins(channelA, contadorA);
  printSpeedEncoder(prevContadorA, contadorA, "Contador(A) = ");
  prevContadorA = contadorA;

  interruptEncoderPins(channelB, contadorB);  
  printSpeedEncoder(prevContadorB, contadorB, "Contador(B) = ");
  prevContadorB = contadorB;    
}

void soma(){
  contador++;
}

void receive() {
   if (radio.available()){
     radio.read(&msg, 4);
     
     analogWrite(PWMA, map(msg[0],0,1023,0,180));
     analogWrite(PWMB, map(msg[1],0,1023,0,180));
  }
}

void send() {
  analogWrite(PWMA, map(300,0,1023,0,180));
  analogWrite(PWMB, map(300,0,1023,0,180));  
  
  encoder();
  //int rpm[2];
  //rpm[0] = ((float)contadorA*100*60)/(512*19);
  //rpm[1] = ((float)contadorB*100*60)/(512*19);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int pot = analogRead(POT);
  
  int dados[9] = {ax, ay, az, gx, gy, gz, pot, contadorA, contadorB};        
  radio.write(&dados, 18);
}

/*int contador_de_envio = 0;
void loop(void){
  
  contador_de_envio++;
  if (contador_de_envio >= 10) {
    contador_de_envio = 0;
    sending = true;
  }
  
  if (sending)
    send();
  else 
    receive();

  sending = false;
}*/

void loop(void){
  if (sending)
    send();
  else 
    receive();
}

