#include <SPI.h>
#include "RF24.h"


/****************** User Config ***************************/
/***      Set this radio as radio number                ***/
int radioNumber = 1; //Central=0

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 2 & 3 */
RF24 radio(3,2);    //arduino nano
//RF24 radio(10,A0);
/**********************************************************/

byte pipe[][6] = {"1Node","2Node","3Node","4Node"};


void setup() {
  Serial.begin(250000);
  Serial.println("Inicio");
  radio.begin();

  if(radio.isPVariant()){
    Serial.println("versao nrf24l01+");
  }
  Serial.println("Insira qualquer caractere");
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);


  /*
  if(radioNumber){
    radio.openWritingPipe(pipe[1]);
    //radio.openReadingPipe(1,pipe[0]);
  }else{
    //
    radio.openReadingPipe(1,pipe[1]);
    // Start the radio listening for data
    radio.startListening();
  }
  */
  //settings for n0 for default
  radio.openReadingPipe(1,pipe[1]);  //cetral->n0 pelo pipe1
  radio.openWritingPipe(pipe[0]);    //n0->central pelo pipe0
  
  // Start the radio listening for data
  radio.startListening();

  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
}
int msg[2]={0,0};
int error=0;
int received=0;
int Nrobos=0;
void loop() {
  if(radioNumber==0){
    //Central apenas recebe dados
    if(radio.available()){
      while (radio.available()) {         
        radio.read(&msg, sizeof(msg));       
      }
      /*
      for(int i=0; i<(sizeof(msg)/sizeof(int)); i++){
        Serial.print(msg[i]);
        Serial.print(" ");
      }
      Serial.println();
      */
      int command=0;
      if(msg[0]==0){
        Nrobos++;
        command=1;//mudar de pipe
        radio.stopListening();
        radio.openWritingPipe(pipe[1]);
        radio.write(&command, sizeof(command));
        radio.startListening();
      }
    }
  }else{
    //Robos
    if(radio.available()){
      while(radio.available()){
        radio.read(&received, sizeof(received));
        if(received==53){
          digitalWrite(7,HIGH);
        }
        if(received==54){
          digitalWrite(7, LOW);
        }
        if(received==55){
          radio.stopListening();
          radio.openReadingPipe(1,pipe[1]);
          msg[0] = 1;
          radio.startListening();
        }
        if(received==56){
          radio.stopListening();
          radio.openReadingPipe(1,pipe[2]);
          msg[0] = 2;
          radio.startListening();
        }
        if(received==57){
          radio.stopListening();
          radio.openReadingPipe(1,pipe[3]);
          msg[0] = 3;
          radio.startListening();
        }
      }
    }
    msg[1] = error;
    radio.stopListening();
    digitalWrite(8, HIGH);
    if(!(radio.write(&msg, sizeof(msg)))){
      error++;
    }
    //error=error+radio.write(&msg, sizeof(msg),0);
    radio.startListening();
    digitalWrite(8, LOW);
  }
  delay(10);
}

void serialEvent() {
  //input no terminal enviado aos receivers
  //inteiro por inteiro
  if(Serial.available()){
    received=Serial.read();
    Serial.print(received);
    if(received==50||received==51){
      if(received==50){
        received=Serial.read();
        Serial.print(received);
        radio.stopListening();
        radio.openWritingPipe(pipe[2]);
        radio.write(&received, sizeof(received));
        radio.startListening();
      }else{
        received=Serial.read();
        Serial.print(received);
        radio.stopListening();
        radio.openWritingPipe(pipe[3]);
        radio.write(&received, sizeof(received));
        radio.startListening();
      }
    }else{
      radio.stopListening();
      radio.openWritingPipe(pipe[1]);
      radio.write(&received, sizeof(received));
      radio.startListening();
    }
    Serial.println();
  }
  if(radioNumber==1){
    Serial.println("Central started");
    radioNumber=0;
    radio.stopListening();
    radio.openWritingPipe(pipe[1]);  //central->n1 pelo pipe1
    radio.openReadingPipe(0,pipe[0]);//n1->central pelo pipe0
    // Start the radio listening for data
    radio.startListening();
  }
}

