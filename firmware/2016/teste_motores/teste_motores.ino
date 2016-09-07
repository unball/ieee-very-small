#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

int msg = 50;

int PWMA=9;
int AIN1=8;
int AIN2=7;

int channelA = 3; //TX

volatile unsigned long contadorA = 0;
volatile unsigned long contador = 0;

RF24 radio(10,A0);  //CE, CS

byte pipe[][6] = {"1Node","2Node","3Node","4Node"};

int LED = 4;
int LED2 = 5;

void setup() {
  radio.begin();

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  radio.openReadingPipe(1,pipe[1]);  //cetral->n0 pelo pipe1
  radio.openWritingPipe(pipe[0]);    //n0->central pelo pipe0
  
  // Start the radio listening for data
  radio.startListening();

  pinMode(channelA, INPUT);  
  setMotorPin(PWMA, AIN1, AIN2);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(LED2, OUTPUT);
}

void setMotorPin(int PWM, int IN1, int IN2) {  
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);  

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
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
}

void soma(){
  contador++;
}

void receive() {
  digitalWrite(LED2, LOW);
   if (radio.available()){  
     while(radio.available()) {
      digitalWrite(LED2, HIGH);
      radio.read(&msg, sizeof(msg));
     }
  }
}

int novo_contador = 0;

void send_message() {
  int dado[2] = {contadorA, msg};
  radio.stopListening();
  digitalWrite(LED, HIGH);        
  radio.write(dado, sizeof(dado));
  radio.startListening();
  digitalWrite(LED, LOW);
}

void move(int speed, int direction) {
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  digitalWrite(AIN1, inPin1);
  digitalWrite(AIN2, inPin2);
  analogWrite(PWMA, speed);
}

void loop() {
  receive();
  int speed = map(msg,-100,100,-255,255);
  move(abs(speed), (speed > 0 ? 1:0));
  encoder();
  send_message();
}
