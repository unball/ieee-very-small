long motorA_direction=0;
long motorB_direction=0;

void motorsSetup(){  
  pinMode(Pins::PWMA, OUTPUT);
  pinMode(Pins::AIN1, OUTPUT);
  pinMode(Pins::BIN1, OUTPUT);

  //pinMode(STBY, OUTPUT);
  
  pinMode(Pins::PWMB, OUTPUT);
  pinMode(Pins::BIN1, OUTPUT);
  pinMode(Pins::BIN2, OUTPUT);
}

void move(int motor, int power) {
  //digitalWrite(STBY, HIGH);
  int pin1, pin2;
  int PWM;
  if (motor == 0) {
    pin1 = Pins::AIN1;
    pin2 = Pins::AIN2;
    PWM = Pins::PWMA;
  }
  else {
    pin1 = Pins::BIN1;
    pin2 = Pins::BIN2;
    PWM = Pins::PWMB;    
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