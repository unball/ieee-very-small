#include <SPI.h>
#include "RF24.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI protocol*/
RF24 radio(3,2);    //arduino nano
//RF24 radio(10,A0); //arduino pro micro
/**********************************************************/

byte pipe[][6] = {"1Node","2Node","3Node","4Node"}; 
int receive_msg[1]; //pipe number

void setup() {
  Serial.begin(250000);
  Serial.println("Inicio");
  radio.begin();
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  radio.openReadingPipe(0,pipe[0]); //n1->central pelo pipe0  
  radio.startListening();
}

void checkIfRadioIsPVariant(){
  if(radio.isPVariant()) {
    Serial.println("versao nrf24l01+");
  }
}

bool receive_data_from_robot() {
  if(radio.available()){
    while (radio.available()) {         
      radio.read(&receive_msg, sizeof(receive_msg));       
    }
    return true;
  }
  return false;
}

void send_data_to_ROS() {
  Serial.println(receive_msg[0]); //Serial.write() may be a better choice for this
  Serial.println(receive_msg[1]); //Serial.write() may be a better choice for this
}

String input_string;
int string_length = 0;

bool receive_data_from_ROS() {
  if(Serial.available()){
    input_string = "";
    while (Serial.available()) {         
      char in_char = (char)Serial.read(); //taken from the SerialEvent example
      input_string += in_char;
    }
    if (input_string.length() == string_length)
      return true;
  }
  return false;
}

void send_data_to_robot() {
  if(radio.available()){
    while (radio.available()) {         
      radio.read(&receive_msg, sizeof(receive_msg));       
    }
    return true;
  }
  return false;  
}

void loop() {
  if (receive_data_from_robot())
    send_data_to_ROS();
  if (receive_data_from_ROS())
    send_data_to_robot();
  
  delay(10);
}

