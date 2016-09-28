#include <SPI.h>
#include "RF24.h"

RF24 radio(10,A0);    //arduino pro micro

byte pipe[][6] = {"1Node","2Node","3Node","4Node"}; 
int send_msg[2]; //number of pipes

int start_pipe = 0;
int my_pipe;

void setPipes(int new_pipe) {
  my_pipe = new_pipe;
  radio.openReadingPipe(1,pipe[my_pipe]);
  radio.openWritingPipe(pipe[my_pipe+1]);    //n0->central pelo pipe0    
}

void setup() {
  Serial.begin(250000);
  while(!Serial);
    Serial.println("Inicio - robo");
  radio.begin();
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  setPipes(start_pipe);
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

bool test_receive() {
  if(radio.available()){
    while (radio.available()) { 
      radio.read(&msg_from_central, sizeof(msg_from_central));       
    }
    Serial.println(msg_from_central[0]);
    Serial.println(msg_from_central[1]);
    return true;
  }
  return false;
}


void set_channel() { 
  radio.stopListening();
  
  setPipes(msg_from_central[1]);
  radio.startListening();  
}

void loop() {
  bool has_received_message = test_receive();
  if (my_pipe == start_pipe) {
    if (has_received_message)
      set_channel();
  }
  delay(10);
}

