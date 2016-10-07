#include <SPI.h>
#include "RF24.h"

int LED = 6;
bool has_received_message;

void setup() {
  radioSetup();
  motorsSetup();
  IMUSetup();
}

void sendMessageBackToCentral() {
    int send_message[2];
    getMessages(send_message);
    send(send_message);   
}

void loop() {
  has_received_message = receive();

  if (has_received_message){
    if (isStartingPipe())
      setChannel();
    else {
      sendMessageBackToCentral(); 
    }    
  }
  
  delay(20);
  has_received_message = false;
}
