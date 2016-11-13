#include "RF24.h"

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

void setChannel() { 
  radio.stopListening();
  
  setPipes(msg_from_central[0]);
  radio.startListening();  
}

void radioSetup() {
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);
  setPipes(starting_pipe);
  radio.startListening();
}

int getMessage(int i) {
  return msg_from_central[i];
}

void getMessages(int *msgs) {
  int i;
  for (i=0; i<RECEIVED_MESSAGE_SIZE; i++)
    msgs[i] = msg_from_central[i];
}

void checkIfRadioIsPVariant() {
  if (radio.isPVariant()) {
    Serial.println("versao nrf24l01+");
  }
  else {
    Serial.println("it is not");
  }
}

bool isStartingPipe() {
  return my_pipe == starting_pipe;
}

void send(int *msg) {
  int i;
  for (i=0;i<SEND_MESSAGE_SIZE;i++)
    send_msg[i] = msg[i];
  
  radio.stopListening();
  
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

