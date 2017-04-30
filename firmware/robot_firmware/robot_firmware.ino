#include <pins.h>
#include <radio.h>
#include <motor.h>
#include <encoder.h>

long errorA_i=0;
long errorB_i=0;
long commandA_media=0;
long commandB_media=0;

int acc=0;

void setup(void) {

  Serial.begin(115200);
  
  radioSetup();
  motorsSetup();
  encodersSetup();

  menu(); //preicsa abrir o serial pelo computador
}

String inString = ""; 

void stand(){
  int acc=0;
  while(true){
    if(radio.available()){
       acc=0; 
       int temp1=velocidades.motorA;
       int temp2=velocidades.motorB;
       while(radio.available()){       
        radio.read(&velocidades,sizeof(velocidades));
       }
       if(!(velocidades.motorA==temp1 && velocidades.motorB==temp2)){
        //errorA_i=0;
        //errorB_i=0;  
       }
     }else{
      //procedimento para indicar que o robo nao recebe mensagens nas ultimas 20000 iteracoes
      acc++;
      if(acc>20000){
        control(200,-200);
      }else{
        control(velocidades.motorA, velocidades.motorB);
      }
    }
  }
}


void loop(){
  stand();
  if (Serial.available()){
    char c = (Serial.read());
    Serial.println();

    switch(c){
      case '1':
      {
        menu();
        Serial.println();
        break;
      }
      case '2':
      {
        Serial.println("Verificando plataforma");
        plataforma();
        break;
      }
      case '3':
      {
        Serial.println("Teste radio");
        if(radio.isPVariant()){
          Serial.println("is PVariant");
        }else{
          Serial.println("it is not PVariant");
        }
        break;
      }
      case '4':
      {
        radio_status();
        break;
      }
      case '5':
      {
        Serial.println(radio.available());
        break;
      }
      case '6':
      {
        mandaMensagem();
        break;
      }
      case '7':
      {
        recebeMensagem();
        break;
      }
      case '8':
      {
        Serial.println("motor: [0,1]");
        while(!Serial.available());
        int motor = int(toupper(Serial.read()))-48;
        Serial.println(motor);
        Serial.println("power: [0-255]");
        // Read serial input:
        while(!Serial.available());
        while (Serial.available() > 0) {
          int inChar = Serial.read();
          if (isDigit(inChar)) {
            // convert the incoming byte to a char
            // and add it to the string:
            inString += (char)inChar;
          }
        }
        Serial.println(inString.toInt());
        move(motor, inString.toInt());
        inString = "";
        break;
      }
      case '9':
      {
        move(0, 0);
        move(1, 0);
        break;
      }
      case '0':
      {
        encoder();
        Serial.print("motor0: ");Serial.print(contadorA);Serial.print("//");Serial.print(contadorA_media);
    Serial.print("  motor1: ");Serial.print(contadorB);Serial.print("//");Serial.println(contadorB_media);
        break;
      }
      case 'a':
      {
         Serial.println("velocidade em ticks por 10 milisegundos:");
         while(true){
          // Read serial input:
          if(Serial.available()){
            inString = "";
            while (Serial.available() > 0) {
              int inChar = Serial.read();
              if (isDigit(inChar)) {
                // convert the incoming byte to a char
                // and add it to the string:
                inString += (char)inChar;
              }
            }
            Serial.println(inString.toInt());
            errorA_i=0;
            errorB_i=0;
          }
          control(inString.toInt(), inString.toInt());          
         }
      }
      case 'b':
      {
        stand();
        break;
      }
      case 'c':
      {
        levelocidades();
        break;
      }
      default:
      {
       Serial.println("opcao nao reconhecida");
       break;
      }
    }
  }
}

void control(int velocidadeA, int velocidadeB){
  if(velocidadeA || velocidadeB){
    encoder();
    Serial.print("  motor0: ");Serial.print(contadorA);Serial.print("//");Serial.print(contadorA_media);
    Serial.print("  motor1: ");Serial.print(contadorB);Serial.print("//");Serial.print(contadorB_media);
    long errorA=velocidadeA-contadorA_media;
    long errorB=velocidadeB-contadorB_media;
    errorA_i+=errorA;
    errorB_i+=errorB;
    
    Serial.print(" error:");
    Serial.print(errorA);
    Serial.print("||");
    Serial.print(errorB);
    
    long ke_a=1200;
    long ki_a=30;
    
    long ke_b=1200;
    long ki_b=30;

    long intermediarioA=0;
    intermediarioA=(ke_a*errorA)/1000;
    intermediarioA+=(ki_a*errorA_i)/1000;

    long intermediarioB=0;
    intermediarioB=(ke_b*errorB)/1000;
    intermediarioB+=(ki_b*errorB_i)/1000;

    int commandA=intermediarioA;
    int commandB=intermediarioB;
    commandA_media+=(commandA-commandA_media)/10;
    commandB_media+=(commandB-commandB_media)/10;

    if(commandA_media>255){
      commandA_media=255;
    }
    if(commandB_media>255){
      commandB_media=255;
    }
    
    move(0, commandA_media);
    move(1, commandB_media);
    Serial.print("   commands ");
    Serial.print(commandA);Serial.print("//");Serial.print(commandA_media);
    Serial.print(" ");Serial.print(commandB);Serial.print("//");Serial.println(commandB_media);
  }else{
    contadorA_media=0;
    contadorB_media=0;
    move(0,0);
    move(1,0);
  }
}

void menu(){
  Serial.println("Menu:");
  Serial.println("1 - menu");
  Serial.println("2 - plataforma");
  Serial.println("3 - teste radio (pvariant test)");
  Serial.println("4 - radio config");
  Serial.println("5 - verificar se existe mensagem");
  Serial.println("6 - mandar mensagem");
  Serial.println("7 - ler mensagem");
  Serial.println("8 - move motor");
  Serial.println("9 - parar todos os motores");
  Serial.println("0 - enconder");
  Serial.println("a - controle de velocidade por ticks por 10 milisegundos");
  Serial.println("b - standAlone");
  Serial.println("c - ler mensagem de velocidade");
}

void plataforma(){
  Serial.println("Plataforma arduino pro micro");
  unsigned long time=millis();
  Serial.println(time);
}

/**********************ENCONDER********************/

