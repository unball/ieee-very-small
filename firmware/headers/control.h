long errorA_i=0;
long errorB_i=0;
long commandA_media=0;
long commandB_media=0;

int acc=0;

void control(int velocidade, int error, long command_media) {
  
}

void control(int velocidadeA, int velocidadeB){
  if(velocidadeA || velocidadeB){
    Encoder::encoder();
    long errorA=velocidadeA-Encoder::contadorA_media;
    errorA_i+=errorA;
    long ke_a=1200;
    long ki_a=30;
    long intermediarioA=0;
    intermediarioA=(ke_a*errorA)/1000;
    intermediarioA+=(ki_a*errorA_i)/1000;    
    int commandA=intermediarioA;
    commandA_media+=(commandA-commandA_media)/10;
    if(commandA_media>255){
      commandA_media=255;
    }
    Motor::move(0, commandA_media);


    long errorB=velocidadeB-Encoder::contadorB_media;
    errorB_i+=errorB;
    long ke_b=1200;
    long ki_b=30;
    long intermediarioB=0;
    intermediarioB=(ke_b*errorB)/1000;
    intermediarioB+=(ki_b*errorB_i)/1000;
    int commandB=intermediarioB;
    commandB_media+=(commandB-commandB_media)/10;
    if(commandB_media>255){
      commandB_media=255;
    }    
    Motor::move(1, commandB_media);
  }else{
    Encoder::contadorA_media=0;
    Encoder::contadorB_media=0;
    Motor::move(0,0);
    Motor::move(1,0);
  }
}

void stand(){
  int acc=0;
  while(true){
    if(Radio::radio.available()){
       acc=0; 
       int temp1=velocidades.motorA;
       int temp2=velocidades.motorB;
       while(Radio::radio.available()){       
        Radio::radio.read(&velocidades,sizeof(velocidades));
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