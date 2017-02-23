/**********FIRMWARE PARA ESTUDO EM CONTROLE***********/
/* http://www-personal.umich.edu/~johannb/Papers/paper43.pdf
 *  Usando a ideia de cross coupling control
  */

  


#include <SPI.h> //for radio
#include "RF24.h" // for radio

/*************  USER Configuration *****************************/
/***********radio***********/
/*Radio pins*/
//int CE = 10; //pro micro
//int CS = A0; //pro micro
int CE = 3; //nano
int CS = 2; //nano

                           
RF24 radio(CE,CS);                        // Set up nRF24L01 radio on SPI bus plus pins 3 & 2 for nano


const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };   // Radio pipe addresses for the 2 nodes to communicate.
uint64_t pipeEnvia=pipes[1];
uint64_t pipeRecebe=pipes[0];


struct dataStruct{
    char data1;
    int data2;
}myData;

struct dataStruct2{
    int motorA=0;
    int motorB=0;
};

String inString = ""; 
/***************************************************************/

dataStruct2 velocidades;

void setup(void) {
  
  
  Serial.begin(115200);
  while(!Serial);

  
  radioSetup();
  
  Serial.println("Firwmare para estudo em controle");
  Serial.println("Objetivo: implementar um cross coupling control");
  Serial.println("Para testar:");
  Serial.println("* radio      OK");
  Serial.println("* enconder");
  Serial.println("* controle");
  Serial.println("##############");
  Serial.println();
  Serial.println();
  Serial.println();


  menu(); //preicsa abrir o serial pelo computador
}

void loop(){
  if (Serial.available()){
    char c = Serial.read();
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
        mandaVelocidades();
        break;
      }
      case '9':
      {
        repeteVelocidade();
        break;
      }
      case '0':
      {
        mandaVelocidadeZero();
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


void mandaVelocidadeZero(){
  velocidades.motorA=0;
  velocidades.motorB=0;
  radio.stopListening();
  radio.enableDynamicAck();
  radio.write(&velocidades,sizeof(velocidades), 1);
  radio.startListening();
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
  Serial.println("8 - mandar velocidades");
  Serial.println("9 - repete a ultima velocidade");
  Serial.println("0 - para o robo");
  Serial.println("a - modoOnline");
  
}

void repeteVelocidade(){
  if(velocidades.motorA || velocidades.motorB){
    Serial.print(velocidades.motorA);
    Serial.print(" ");
    Serial.print(velocidades.motorB);
    Serial.print(" ");
    radio.stopListening();
    radio.enableDynamicAck();
    radio.write(&velocidades,sizeof(velocidades), 1);
    radio.startListening();
  }
}

void mandaVelocidades(){
  Serial.print("Velocidade motor A:");
  while(!Serial.available());
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    delay(30);
  }
  velocidades.motorA=inString.toInt();
  Serial.println(velocidades.motorA);
  inString = "";
  Serial.print("Velocidade motor B:");
  while(!Serial.available());
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    delay(30);
  }
  velocidades.motorB=inString.toInt();
  inString = "";
  Serial.println(velocidades.motorB);

  //velocidades.motorA*=-1;
  //velocidades.motorB*=-1;
  radio.stopListening();
  radio.enableDynamicAck();
  radio.write(&velocidades,sizeof(velocidades), 1);
  radio.startListening();
  delay(50);
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
  Serial.println("Plataforma arduino nano");
  unsigned long time=millis();
  Serial.println(time);
}

void radioSetup(){
  radio.begin();                           // inicializa radio
  radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
  radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
  radio.setDataRate(RF24_2MBPS);           //usa velocidade maxima

  radio.openWritingPipe(pipeEnvia);        //escreve pelo pipe0
  radio.openReadingPipe(1,pipeRecebe);      //escuta pelo pipe1

  radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
  radio.setPayloadSize(sizeof(velocidades));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
  
  radio.startListening();                 // Start listening
}


