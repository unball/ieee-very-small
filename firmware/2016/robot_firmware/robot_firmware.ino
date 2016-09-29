#include <SPI.h>
#include "RF24.h"

/*Radio pins depend on the Arduino used*/
RF24 radio(10,A0);    //arduino pro micro
//RF24 radio(3, 2);   //arduino nano

/*The starting LISTENING pipe defines the "lost robots" zone. The communication between
the central and the robots will not be made through this pipe. This pipe will 
only be used to tell the robot which pipe it should switch to*/
int starting_pipe = 0;

/*Actual pipe the robot will use to LISTEN. The writting pipe number is, by default, 
 * the listening pipe number + 1 (my_pipe+1)*/
int my_pipe;

byte pipe[][6] = {"1Node","2Node","3Node","4Node"};

#define SEND_MESSAGE_SIZE 2
int send_msg[SEND_MESSAGE_SIZE];  //The message the robot will send to the central

#define RECEIVED_MESSAGE_SIZE 2
int msg_from_central[RECEIVED_MESSAGE_SIZE] = {0,0};

/* 
 * Listening Pipe <- new_pipe
 * Writting Pipe <- new_pipe + 1
 */
void setPipes(int new_pipe) {
  my_pipe = new_pipe;
  radio.openReadingPipe(1,pipe[my_pipe]);
  radio.openWritingPipe(pipe[my_pipe+1]);   
}

void radioSetup() {
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);
}

void setup() {
  radio.begin();
  radioSetup();
  
  setPipes(starting_pipe);
  radio.startListening();
}

void send() {
  radio.stopListening();
  int i;
  for (i=0;i<SEND_MESSAGE_SIZE;i++) {
    radio.write(send_msg, sizeof(send_msg));      
  }
  radio.startListening();
}

bool receive() {
  if(radio.available()){
    while (radio.available()) { 
      radio.read(&msg_from_central, sizeof(msg_from_central));       
    }
    return true;
  }
  return false;
}

void set_channel() { 
  radio.stopListening();
  
  setPipes(msg_from_central[0]);
  radio.startListening();  
}

void loop() {
  bool has_received_message = receive();
  if (my_pipe == starting_pipe) {
    if (has_received_message)
      set_channel();
  }
  delay(10);
}
