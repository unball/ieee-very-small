/*
Code for Transmitter
http://www.instructables.com/id/Wireless-Remote-Using-24-Ghz-NRF24L01-Simple-Tutor
*/

#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

bool sending = false;
int msg[9];

RF24 radio(10,A0); //pro micro

const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void){
    Serial.begin(115200);
    Serial.println("Starting");
    radio.begin();

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    
    //radio.openWritingPipe(pipe);
}

void send() {
    msg[0] = analogRead(A2);
    msg[1] = msg[0]/2;
    radio.write(&msg, 4);

   Serial.print(msg[0]);
   Serial.print("   ");
   Serial.println(msg[1]);  
}

void receive() {
  radio.openReadingPipe(0,pipe);
  radio.startListening();
  if (radio.available()) {
    radio.read(&msg,18);
    Serial.print("Acelerometro: ");
    Serial.print(msg[0]);
    Serial.print(" ");
    Serial.print(msg[1]);
    Serial.print(" ");
    Serial.print(msg[2]);
    Serial.print(" ");
    Serial.print(msg[3]);
    Serial.print(" ");
    Serial.print(msg[4]);
    Serial.print(" ");
    Serial.println(msg[5]);
    
    Serial.print("Potenciometro: ");
    Serial.println(msg[6]);

    Serial.print("Encoder: ");
    Serial.print(msg[7]);
    Serial.print(" ");
    Serial.println(msg[8]);
  }
}

void loop(void){
  if (sending)
    send();
  else 
    receive();
}
