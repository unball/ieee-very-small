#ifndef RADIO_H
#define RADIO_H
#include <pins.h>


#include <SPI.h>
#include "RF24.h"

struct dataStruct{
    int A=0;
    int B=0;
};

dataStruct velocidades, report;

namespace Radio {
  RF24 radio(Pins::CE,Pins::CS);

  const uint64_t pipes[4] = { 0xABCDABCD71LL, 0x544d52687CLL, 0x644d52687CLL, 0x744d52687CLL };
  uint64_t pipeEnvia=pipes[0];
  uint64_t pipeRecebe=pipes[robot_number];

  void Setup(){
    radio.begin();                           // inicializa radio
    radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
    radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
    radio.setDataRate(RF24_2MBPS);           //usa velocidade de transmissao maxima

    radio.openWritingPipe(pipeEnvia);        //escreve pelo pipe0 SEMPRE
    radio.openReadingPipe(1,pipeRecebe);      //escuta pelo pipe1, evitar usar pipe0

    radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
    radio.setPayloadSize(sizeof(velocidades));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
                                                 //tamanho maximo de payload 32 BYTES
    
    radio.startListening();                 // Start listening
  }

  void levelocidades(){
    if(radio.available()){
       while(radio.available()){       
        radio.read(&velocidades,sizeof(velocidades));
       }
    Serial.println(velocidades.A);
    Serial.println(velocidades.B);
    }
  }

  bool receivedata(dataStruct *data){ // recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false
     if(radio.available()){      
        radio.read(data,sizeof(dataStruct));
        return true;
     }
    return false;
  }

  void reportMessage(int message){
    report.A = robot_number;
    report.B = message;
    radio.stopListening();
    radio.enableDynamicAck();                 //essa funcao precisa andar colada na de baixo
    radio.write(&report, sizeof(report), 1);  //lembrar que precisa enableDynamicAck antes
                                              // 1-NOACK, 0-ACK
    radio.startListening();
  }

  void mandaMensagem(){
    Serial.println("Escreva 1 caractere");
    while(!Serial.available());
    
    char c = toupper(Serial.read());
    velocidades.A=c;
    radio.stopListening();
    radio.enableDynamicAck();                 //essa funcao precisa andar colada na de baixo
    radio.write(&velocidades,sizeof(velocidades), 1);   //lembrar que precisa enableDynamicAck antes
                                              // 1-NOACK, 0-ACK
    radio.startListening(); 
    Serial.println(c);
  }

  void radio_status(){
    Serial.print("CE: ");
    Serial.println(Pins::CE);
    Serial.print("CS: ");
    Serial.println(Pins::CS);
    
    Serial.print("Channel: ");
    Serial.print(radio.getChannel());
    Serial.println("  0-125");
    Serial.print("PALevel: ");
    Serial.print(radio.getPALevel());
    Serial.println("  0-3");
    Serial.print("DataRate: ");
    Serial.print(radio.getDataRate());
    Serial.println("  0->1MBPS, 1->2MBPS, 2->250KBPS");
    Serial.println();
    Serial.print("Enviar por: ");
    Serial.println(int(pipeEnvia));
    Serial.print("Recebe por: ");
    Serial.println(int(pipeRecebe));
    Serial.print("Payload size: ");
    Serial.println(radio.getPayloadSize());
  }
}

#endif