#include <SPI.h>
#include "RF24.h"

RF24 radio(10,A0);    //arduino pro micro

byte pipe[][6] = {"1Node","2Node","3Node","4Node"}; 
int send_msg[2]; //number of pipes

void setup() {
  Serial.begin(250000);
  while(!Serial);
    Serial.println("Inicio - robo");
  radio.begin();
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  radio.openReadingPipe(1,pipe[1]);
  radio.openWritingPipe(pipe[0]);    //n0->central pelo pipe0  

  radio.startListening();
}

int contador = 0;

void test_send() {
  radio.stopListening();
  
  send_msg[0] = contador++;
  send_msg[1] = -contador;
  Serial.println(contador);
  radio.write(send_msg, sizeof(send_msg));       

  radio.startListening();
}

int msg_from_central[2] = {0,0};

void test_receive() {
  if(radio.available()){
    while (radio.available()) { 
      radio.read(&msg_from_central, sizeof(msg_from_central));       
    }
    Serial.println(msg_from_central[0]);
    Serial.println(msg_from_central[1]);
  }
}

void loop() {
  test_receive();
  delay(10);
}

