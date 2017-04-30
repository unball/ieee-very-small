long motorA_direction=0;
long motorB_direction=0;

void motorsSetup(){  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);

  //pinMode(STBY, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
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