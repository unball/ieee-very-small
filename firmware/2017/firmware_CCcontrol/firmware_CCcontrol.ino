/**********FIRMWARE PARA ESTUDO EM CONTROLE***********/
/* http://www-personal.umich.edu/~johannb/Papers/paper43.pdf
 *  Usando a ideia de cross coupling control
  */

/*Encoder pins*/
int channelA = 3; //TX = 3
int channelB = 2; //RX = 2

/*Motor pins*/
int PWMA = 9;
int AIN1 = 7;
int AIN2 = 8;

//int STBY = A3;

int PWMB = 6;//10
int BIN1 = 5;
int BIN2 = 4;

volatile long contadorA = 0;
volatile long contadorB = 0;
volatile long contador = 0;
long contadorA_media=0;
long contadorB_media=0;
long motorA_direction=0;
long motorB_direction=0;
long errorA_i=0;
long errorB_i=0;
long commandA_media=0;
long commandB_media=0;
#include <SPI.h> //for radio
#include "RF24.h" // for radio

int acc=0;

/*************  USER Configuration *****************************/
/***********radio***********/
/*Radio pins*/
int CE = A0; //pro micro
int CS = 10; //pro micro
//int CE = 3; //nano
//int CS = 2; //nano

                           
RF24 radio(CE,CS);                        // Set up nRF24L01 radio on SPI bus plus pins 3 & 2 for nano


const uint64_t pipes[4] = { 0xABCDABCD71LL, 0x544d52687CLL, 0x644d52687CLL, 0x744d52687CLL};
uint64_t pipeEnvia=pipes[0];
uint64_t pipeRecebe=pipes[3];


struct dataStruct{
    char data1;
    int data2;
}myData;

struct dataStruct2{
    int motorA=0;
    int motorB=0;
}velocidades;


/***************************************************************/


void setup(void) {

  Serial.begin(115200);
  Serial.println("Inicio");
  
  
  radioSetup();
  motorsSetup();
  encodersSetup();
  
  Serial.println("Firwmare para estudo em controle");
  Serial.println("Objetivo: implementar um cross coupling control");
  Serial.println("Para testar:");
  Serial.println("* radio      OK");
  Serial.println("* enconder    OK");
  Serial.println("* controle");
  Serial.println("##############");
  Serial.println();
  Serial.println();
  Serial.println();
  

  menu(); //preicsa abrir o serial pelo computador
  
}

String inString = ""; 
//loop para controlar a radio

void stand(){
  int acc=0;
  //Serial.println("aguardando mensagem, resete para retirar desse modo");
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
        control(700,-700);
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

void levelocidades(){
  if(radio.available()){
     while(radio.available()){       
      radio.read(&velocidades,sizeof(velocidades));
     }
  Serial.println(velocidades.motorA);
  Serial.println(velocidades.motorB);
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
    
    long ke_a=1400;
    long ki_a=5;
    
    long ke_b=1400;
    long ki_b=5;

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

void recebeMensagem(){
   if(radio.available()){
     while(radio.available()){       
      radio.read(&myData,sizeof(myData));
     }
     Serial.print("mensagem: ");
     Serial.println(myData.data1);
   }
}

void mandaMensagem(){
  Serial.println("Escreva 1 caractere");
  while(!Serial.available());
  
  char c = toupper(Serial.read());
  myData.data1=c;
  radio.stopListening();
  radio.enableDynamicAck();                 //essa funcao precisa andar colada na de baixo
  radio.write(&myData,sizeof(myData), 1);   //lembrar que precisa enableDynamicAck antes
                                            // 1-NOACK, 0-ACK
  radio.startListening(); 
  Serial.println(c);
}

void radio_status(){
  Serial.print("CE: ");
  Serial.println(CE);
  Serial.print("CS: ");
  Serial.println(CS);
  
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

void plataforma(){
  Serial.println("Plataforma arduino pro micro");
  unsigned long time=millis();
  Serial.println(time);
}

void move(int motor, int power) {
  //digitalWrite(STBY, HIGH);
  int pin1, pin2;
  int PWM;
  if (motor == 0) {
    pin1 = AIN1;
    pin2 = AIN2;
    PWM = PWMA;
  }
  else {
    pin1 = BIN1;
    pin2 = BIN2;
    PWM = PWMB;    
  }
  
  //power = map(power,-100,100,-255,255);
  if(power>255) {
    power=255;
    Serial.print("SATURADO!");
  }
  if(power<-255){
    power=-255;
    Serial.print("SATURADO!");
  }
  int power_magnitude = abs(power);
  if(motor==0){
    motorA_direction=(power > 0 ? 1:-1);
  }else{
    motorB_direction=(power > 0 ? 1:-1);
  }
  int motor_direction = (power > 0 ? 0:1);

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (motor_direction == 0) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  digitalWrite(pin1, inPin1);
  digitalWrite(pin2, inPin2);
  analogWrite(PWM, abs(power));
}

void motorsSetup(){  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN1, OUTPUT);

  //pinMode(STBY, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
}

void radioSetup(){
  radio.begin();                           // inicializa radio
  radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
  radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
  radio.setDataRate(RF24_2MBPS);           //usa velocidade de transmissao maxima

  radio.openWritingPipe(pipeEnvia);        //escreve pelo pipe0
  radio.openReadingPipe(1,pipeRecebe);      //escuta pelo pipe1

  radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
  radio.setPayloadSize(sizeof(velocidades));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
  
  radio.startListening();                 // Start listening
}

/**********************ENCONDER********************/


void encodersSetup() {
  pinMode(channelA, INPUT);  
  pinMode(channelB, INPUT);
}

void soma(){
  contador++;
}

void interruptEncoderPins(int channel, volatile long &contador_i) {
  contador = 0;
  attachInterrupt(channel, soma, RISING);
  delay(10);
  detachInterrupt(channel);
  contador_i = contador;  
}

void encoder() {
  interruptEncoderPins(channelA, contadorA);
  interruptEncoderPins(channelB, contadorB);

  contadorA_media+=(motorA_direction*contadorA-contadorA_media)/10;
  contadorB_media+=(motorB_direction*contadorB-contadorB_media)/10;
}
