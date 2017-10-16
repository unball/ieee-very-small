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
  int sat_count = 0;
  unsigned long cicle_time=0;
  bool bateria_fraca;
  int tensao=0;

  int acc=0;


  void stopRobot() {
      Motor::stop(0);
      Motor::stop(1);
  }

  bool Bateria(long ki_erroA, long ki_erroB, long Sat_ki_erro, int velA, int velB){
    if(abs(ki_erroA) > Sat_ki_erro || abs(ki_erroB) > Sat_ki_erro){
      sat_count++;
    }
    else{
      sat_count = 0;
    }

    if(sat_count > 50){
      if(abs(velA) < 600 || abs(velB) < 600){
        Serial.print("VERIFICAR BATERIA DO ROBO ");
        Serial.println(robot_number);
        return true;
      }
      else{
        return false;
      }
    }
  }

  //verifica e imprime o tempo de duração de um ciclo
  void TimeOfCicle(){
    cicle_time = millis() - cicle_time;
    Serial.print("TIME: ");
    Serial.println(cicle_time);
    cicle_time = millis();
  }

  void control(int velocidadeA, int velocidadeB){
    if(velocidadeA || velocidadeB){
    Encoder::encoder();
    //TimeOfCicle();
    Serial.print("motor0: ");Serial.print(Encoder::contadorA);Serial.print("//");Serial.print(Encoder::contadorA_media);
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

    long kp_a=2100;
    long ki_a=0;
    long kd_a=1800;
    
    long kp_b=2100;
    long ki_b=0;
    long kd_b=1800;

    long Saturacao_ki_erro=200;

    long intermediarioA=0;
    intermediarioA=(ki_a*errorA_i)/1000;
    long ki_erro_A = intermediarioA;

    //Saturação do ki*errorA_i
    if(intermediarioA > Saturacao_ki_erro){
      errorA_i = 1000*Saturacao_ki_erro/ki_a;
    }
    else if(intermediarioA < (-1)*Saturacao_ki_erro){
      errorA_i = (-1000)*Saturacao_ki_erro/ki_a;
    }
    intermediarioA=(ki_a*errorA_i)/1000;
    //Serial.print("  errorA_i: ");Serial.print(errorA_i);
    intermediarioA += (kp_a*errorA)/1000;
    intermediarioA += (kd_a*errorA_d)/1000;
    errorA_d_ant = errorA;


    long intermediarioB=0;
    intermediarioB=(ki_b*errorB_i)/1000;
    long ki_erro_B = intermediarioB;

    //Saturação do ki*errorB_i
    if(intermediarioB > Saturacao_ki_erro){
      errorB_i = 1000*Saturacao_ki_erro/ki_b;
    }
    else if(intermediarioB < (-1)*Saturacao_ki_erro){
      errorB_i = (-1000)*Saturacao_ki_erro/ki_b;
    }
    intermediarioB=(ki_b*errorB_i)/1000;
    //Serial.print("  errorB_i: ");Serial.print(errorB_i);
    intermediarioB += (kp_b*errorB)/1000;
    intermediarioB += (kd_b*errorB_d)/1000;
    errorB_d_ant = errorB;
    

    int commandA=intermediarioA;
    int commandB=intermediarioB;

    if(commandA > 255){
      commandA = 255;
    }
    else if(commandA < -255){
      commandA = -255;
    }
    if(commandB > 255){
      commandB = 255;
    }
    else if(commandB < -255){
      commandB = -255;
    }


    //Teste para verificação dos motores
    /*tensao++;
    if(tensao>220){
      tensao=220;
      Serial.println("#");
    }
    Serial.println("$");
    Serial.println(tensao);
    Serial.println(Encoder::contadorA_media);
    Serial.println(Encoder::contadorB_media);*/

    Motor::move(0, commandA);
    Motor::move(1, commandB);
    
    Serial.print("   commands ");
    Serial.print(commandA);Serial.print("//");
    Serial.print(" ");Serial.println(commandB);

    //delay(300);

    //verifica se o erro integrativo satura a velocidade baixa
    //bateria_fraca = Bateria(ki_erro_A, ki_erro_B, Saturacao_ki_erro, velocidadeA, velocidadeB);

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
    //procedimento para indicar que o robo nao recebe mensagens nas ultimas 20000 iteracoes
    else {
      if(radioNotAvailableFor(20000)){
        control(600, 600);
      }
      else {
        //control(500, 500);
        control(velocidades.motorA, velocidades.motorB);
      }
    }
  }  

}//end namespace

#endif 