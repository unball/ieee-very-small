#ifndef CONTROL_H
#define CONTROL_H
#include <motor.h>
#include <encoder.h>
#include <radio.h>
#define MOTOR_TEST false   //define se está ou não fazendo o teste nos motores
#define wave 2    // sine = 1 -- square = 2 -- step = 3
#define PI 3.14159265


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
  int acc=0;

  //variaveis de teste
  int wave_flag=1;
  int angulo=0;
  long square_cont=0, cont=0;


  void stopRobot() {
      Motor::stop(0);
      Motor::stop(1);
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
    if(!MOTOR_TEST){
      Serial.print("motor0: ");Serial.print(Encoder::contadorA);Serial.print("//");Serial.print(Encoder::contadorA_media);
      Serial.print("  motor1: ");Serial.print(Encoder::contadorB);Serial.print("//");Serial.print(Encoder::contadorB_media);
    }
    long errorA=velocidadeA-Encoder::contadorA_media;
    long errorB=velocidadeB-Encoder::contadorB_media;
    errorA_i+=errorA;
    errorB_i+=errorB;

    long errorA_d = errorA - errorA_d_ant;
    long errorB_d = errorB - errorB_d_ant;
    
    if(!MOTOR_TEST){
      Serial.print(" error:");
      Serial.print(errorA);
      Serial.print("||");
      Serial.print(errorB);
    }

    long kp_a=2100;
    long ki_a=5;
    long kd_a=1800;
    
    long kp_b=2100;
    long ki_b=5;
    long kd_b=2000;

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

    float km = 0.9;
    commandA_media = km*commandA_media + (1-km)*commandA;
    commandB_media = km*commandB_media + (1-km)*commandB;

    //commandA = commandA_media;
    //commandB = commandB_media;


    //Teste para verificação dos motores
    if(MOTOR_TEST){
      Serial.println("$");
      Serial.println(velocidadeA);
      Serial.println(Encoder::contadorA_media);
      Serial.println(Encoder::contadorB_media);
    }



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

    Motor::move(0, commandA);
    Motor::move(1, commandB);

    if(!MOTOR_TEST){
      Serial.print("   commands ");
      Serial.print(commandA);Serial.print("//");
      Serial.print(" ");Serial.println(commandB);
    }

    //verifica se o erro integrativo satura a velocidade baixa
    //bateria_fraca = Bateria(ki_erro_A, ki_erro_B, Saturacao_ki_erro, velocidadeA, velocidadeB);

    }else{
      Encoder::resetEncoders();
      stopRobot();
    }
  }

  bool radioNotAvailableFor(int numberOfCicles) {
    if(acc<numberOfCicles+10)
      acc++;
    return acc>numberOfCicles;
  } 

  void stand() {
    if(Radio::receivedata(&velocidades)) { 
       acc=0;   
    }
    //procedimento para indicar que o robo nao recebe mensagens nas ultimas 20000 iteracoes
    if(radioNotAvailableFor(20000)){
      //Radio::reportMessage(1);
      int vA=500, vB=-500;
      bool asd;

      if(MOTOR_TEST){
        if(wave == 1){
          angulo++;
          if(angulo>720*4){
            Serial.println("#");
          }
          vA = 700*sin(angulo*(PI/180)/4);
          vB = 700*sin(angulo*(PI/180)/4);
        }
        else if(wave == 2){
          cont+=1;
          if(square_cont > 8){
            Serial.println("#");
          }
          if(cont>100){
            wave_flag = -1*wave_flag;
            cont = 0;
            square_cont+=1;
          }
          vA = vA*wave_flag;
          vB = -1*vB*wave_flag;
        }
        else if(wave == 3){
          vA = 500;
          vB = 500;
        }
      }
      control(vA, vB);
    }
    else {
      //control(500, 500);
      control(velocidades.A, velocidades.B);
    }
  }  

}//end namespace

#endif 