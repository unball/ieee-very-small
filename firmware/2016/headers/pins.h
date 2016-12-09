#ifndef Pins_h
#define Pins_h

/*Motor pins*/
int PWMA = 9;
int AIN1 = 8;
int AIN2 = 7;

int PWMB = 6;
int BIN1 = 4;
int BIN2 = 5;

int STDBY;

/*Encoder pins*/
int channelA = 3; //TX = 3
int channelB = 2; //RX = 2

/*Radio pins*/
int CE = 10; //pro micro
int CS = A0; //pro micro
//int CE = 3; //nano
//int CS = 2; //nano

#endif
