#ifndef CONTROL_H
#define CONTROL_H
#include <motor.h>
#include <encoder.h>
#include <radio.h>


namespace Control {

  long errorA_i=0;
  long errorB_i=0;
  long errorA_d_ant=0;
  long errorB_d_ant=0;
  long commandA_media=0;
  long commandB_media=0;

  int acc=0;


  void stopRobot() {
      Motor::stop(0);
      Motor::stop(1);
  }

  void control(int velocidadeA, int velocidadeB){
    if(velocidadeA || velocidadeB){
    Encoder::encoder();
    Serial.print("  motor0: ");Serial.print(Encoder::contadorA);Serial.print("//");Serial.print(Encoder::contadorA_media);
    Serial.print("  motor1: ");Serial.print(Encoder::contadorB);Serial.print("//");Serial.print(Encoder::contadorB_media);
    long errorA=velocidadeA-Encoder::contadorA_media;
    long errorB=velocidadeB-Encoder::contadorB_media;
    errorA_i+=errorA;
    errorB_i+=errorB;

    long errorA_d = errorA - errorA_d_ant;
    long errorB_d = errorB - errorB_d_ant;
    
    Serial.print(" error:");
    Serial.print(errorA);
    Serial.print("||");
    Serial.print(errorB);
    
    long kp_a=1200;
    long ki_a=70;
    long kd_a=12000;
    
    long kp_b=1200;
    long ki_b=70;
    long kd_b=12000;

    long Saturacao_ki_erro=200;

    long intermediarioA=0;
    intermediarioA=(ki_a*errorA_i)/1000;

    //Saturação do ki*errorA_i
    if(intermediarioA > Saturacao_ki_erro){
      errorA_i = 1000*Saturacao_ki_erro/ki_a;
    }
    else if(intermediarioA < (-1)*Saturacao_ki_erro){
      errorA_i = (-1000)*Saturacao_ki_erro/ki_a;
    }
    intermediarioA=(ki_a*errorA_i)/1000;
    Serial.print("  errorA_i: ");Serial.print(errorA_i);
    intermediarioA += (kp_a*errorA)/1000;
    intermediarioA += (kd_a*errorA_d)/1000;
    errorA_d_ant = errorA;


    long intermediarioB=0;
    intermediarioB=(ki_b*errorB_i)/1000;

    //Saturação do ki*errorB_i
    if(intermediarioB > Saturacao_ki_erro){
      errorB_i = 1000*Saturacao_ki_erro/ki_b;
    }
    else if(intermediarioB < (-1)*Saturacao_ki_erro){
      errorB_i = (-1000)*Saturacao_ki_erro/ki_b;
    }
    intermediarioB=(ki_b*errorB_i)/1000;
    Serial.print("  errorB_i: ");Serial.print(errorB_i);
    intermediarioB += (kp_b*errorB)/1000;
    intermediarioB += (kd_b*errorB_d)/1000;
    errorB_d_ant = errorB;
    

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
    
    Motor::move(0, commandA_media);
    Motor::move(1, commandB_media);
    Serial.print("   commands ");
    Serial.print(commandA);Serial.print("//");Serial.print(commandA_media);
    Serial.print(" ");Serial.print(commandB);Serial.print("//");Serial.println(commandB_media);
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
        control(500, 500);
      else {
        control(velocidades.motorA, velocidades.motorB);
      }
    }
  }  
}

#endif