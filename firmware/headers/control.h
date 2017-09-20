namespace Control {

  long errorA_i=0;
  long errorB_i=0;
  long commandA_media=0;
  long commandB_media=0;

  int acc=0;

  void control(int velocidade, int error_i, long command_media, long contador_media) {
    long error=velocidade-contador_media;
    error_i+=error;
    long ke=1200;
    long ki=30;
    long intermediario=0;
    intermediario=(ke*error)/1000;
    intermediario+=(ki*error_i)/1000;    
    int command=intermediario;
    command_media+=(command-command_media)/10;
    
    if(command_media>255)
      command_media=255;
    Motor::move(0, command_media);
  }

  void stopRobot() {
      Motor::stop(0);
      Motor::stop(1);
  }

  void control(int velocidadeA, int velocidadeB){
    if(velocidadeA || velocidadeB){
      Encoder::encoder();
      control(velocidadeA, errorA_i, commandA_media, Encoder::contadorA_media);
      control(velocidadeB, errorB_i, commandB_media, Encoder::contadorB_media);
    }else{
      Encoder::resetEncoders();
      stopRobot();
    }
  }

  bool radioNotAvailableFor(int numberOfCicles) {
    acc++;
    return acc>numberOfCicles;
  }

  void stand() {
    if(Radio::radio.available()) {
       acc=0;
       while(Radio::radio.available()) {       
        Radio::radio.read(&velocidades,sizeof(velocidades));
       }
     }
     else {
      //procedimento para indicar que o robo nao recebe mensagens nas ultimas 20000 iteracoes
      if(radioNotAvailableFor(20000))
        control(200,-200);
      else 
        control(velocidades.motorA, velocidades.motorB);
    }
  }  
}