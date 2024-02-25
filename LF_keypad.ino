
//Infra red
const int irPin = 3;  
int irValue = 0;   

int mode = 0;

//Motor tb6612fng
int pwmRight = 9;
int mRight = 5; 
int dirRight = 4;
int pwmLeft = 10;
int mLeft = 6; 
int dirLeft = 7;

//Line sensor
int sensor[5] = {0,0,0,0,0};
int error, last_error, MV,pid_l,pid_r,D,D1,D2,D3,I,I1,I2,I3,P,Pd, bitSensor;
int Max_MV;
unsigned char Kp = 30; //10;
unsigned char Kd = 5;
unsigned char Ki = 2;
unsigned char Ts = 1;
unsigned char maxPwm = 125;

//count intersection and junction
int countIntersec = 0;
int countLeftJunc = 0;
int countRightJunc = 0;
byte next=0;

void(* resetFunc) (void) = 0;

void setup(){
  Serial.begin(9600);
  
  //Motor tb6612fng
  pinMode(pwmRight, OUTPUT);
  pinMode(mRight, OUTPUT); 
  pinMode(dirRight, OUTPUT); 
  pinMode(pwmLeft, OUTPUT);
  pinMode(mLeft, OUTPUT); 
  pinMode(dirLeft, OUTPUT); 
  //Line sensor
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(A4, INPUT);
  pinMode(A6, INPUT);
  pinMode(A8, INPUT);
  pinMode(A1, INPUT);

  //Infra red
  pinMode(irPin, INPUT);    
}

void loop(){
  mission();     
}

void readSensor(){
  sensor[0] = digitalRead(A0); sensor[1] = digitalRead(A2); sensor[2] = digitalRead(A4);
  sensor[3] = digitalRead(A6); sensor[4] = digitalRead(A8); 
}

void goDrive(){   
  readSensor();
  bitSensor = ((sensor[0]*1)+(sensor[1]*2)+(sensor[2]*4)+(sensor[3]*8)+(sensor[4]*16));
  switch(bitSensor){
    case 0b01111: error = 4; break;
    case 0b00111: error = 3; break;
    case 0b10111: error = 2; break;
    case 0b10011: error = 1; break;
    case 0b11011: error = 0; break;
    case 0b11001: error = -1; break;
    case 0b11101: error = -2; break;
    case 0b11100: error = -3; break;
    case 0b11110: error = -4; break;
  }
  
  Max_MV = Kp*6; 
  P = Kp * error;
  D1 = Kd*8;
  D2 = D1 / Ts;
  D3 = error - last_error;
  D = D2 * D3;

  I1 = Ki/8;
  I2 = error + last_error;
  I3 = I1 * I2;
  I = I3 * Ts; 
  last_error = error;
  
  Pd = P + D;
  MV = P + I;
  
  if(MV>=-Max_MV && MV<=Max_MV){ 
    pid_l = maxPwm + MV;
    pid_r = maxPwm - MV;
    if (pid_l < 0) pid_l = 0;
    if (pid_l > 255) pid_l = 255;
    if (pid_r < 0) pid_r = 0;
    if (pid_r > 255) pid_r = 255;
    forward(pid_r,pid_l);
  }
  else if(MV<-Max_MV){
    turnLeft(100,200);
  }
  else if(MV>Max_MV){
    turnRight(200,100);
  }
  else{
    forward(pid_r,pid_l);
  }
}

void forward(int valLeft, int valRight){
  digitalWrite(mRight, LOW);
  digitalWrite(dirRight, HIGH);
  analogWrite(pwmRight, valRight);
  digitalWrite(mLeft, HIGH);
  digitalWrite(dirLeft, LOW);
  analogWrite(pwmLeft, valLeft);
}

void backward(int valLeft, int valRight){
  digitalWrite(mRight, HIGH);
  digitalWrite(dirRight, LOW);
  analogWrite(pwmRight, valRight);
  digitalWrite(mLeft, LOW);
  digitalWrite(dirLeft, HIGH);
  analogWrite(pwmLeft, valLeft);
  while(sensor[1]==0){readSensor();}
  while(sensor[2]==1){readSensor();}
  while(sensor[3]==0){readSensor();}
} 


void stright1(int valLeft, int valRight){
  digitalWrite(mRight, LOW);
  digitalWrite(dirRight, HIGH);
  analogWrite(pwmRight, valRight);
  digitalWrite(mLeft, HIGH);
  digitalWrite(dirLeft, LOW);
  analogWrite(pwmLeft, valLeft);
  while(sensor[4]==0){readSensor();}
}

void stright2(int valLeft, int valRight){
  digitalWrite(mRight, LOW);
  digitalWrite(dirRight, HIGH);
  analogWrite(pwmRight, valRight);
  digitalWrite(mLeft, HIGH);
  digitalWrite(dirLeft, LOW);
  analogWrite(pwmLeft, valLeft);
  while(sensor[0]==0){readSensor();}
}

void turnRight(int valLeft, int valRight){ 
  digitalWrite(mRight, LOW);
  digitalWrite(dirRight, HIGH);
  analogWrite(pwmRight, valRight);
  digitalWrite(mLeft, LOW);
  digitalWrite(dirLeft, HIGH);
  analogWrite(pwmLeft, valLeft);
  while(sensor[3]==1){readSensor();}
  while(sensor[2]==0){readSensor();}
  while(sensor[1]==1){readSensor();} 
}

void turnLeft(int valLeft, int valRight){
  digitalWrite(mRight, HIGH);
  digitalWrite(dirRight, LOW);
  analogWrite(pwmRight, valRight);
  digitalWrite(mLeft, HIGH);
  digitalWrite(dirLeft, LOW);
  analogWrite(pwmLeft, valLeft);
  while(sensor[1]==0){readSensor();}
  while(sensor[2]==1){readSensor();}
  while(sensor[3]==0){readSensor();}
}

void turnBack(int valLeft, int valRight){ 
  digitalWrite(mRight, LOW);
  digitalWrite(dirRight, HIGH);
  analogWrite(pwmRight, valRight);
  digitalWrite(mLeft, LOW);
  digitalWrite(dirLeft, HIGH);
  analogWrite(pwmLeft, valLeft);
  while(sensor[4]==1){readSensor();}
  while(sensor[3]==1){readSensor();}
  while(sensor[2]==1){readSensor();}
  while(sensor[1]==0){readSensor();}
}

void stopDrive(){
  digitalWrite(dirRight, HIGH);
  digitalWrite(mRight, HIGH);
  analogWrite(pwmRight, 0);
  digitalWrite(dirLeft, HIGH);
  digitalWrite(mLeft, HIGH);
  analogWrite(pwmLeft, 0);  
}

void endDrive(){
  digitalWrite(dirRight, HIGH);
  digitalWrite(mRight, HIGH);
  analogWrite(pwmRight, 0);
  digitalWrite(dirLeft, HIGH);
  digitalWrite(mLeft, HIGH);
  analogWrite(pwmLeft, 0); 
  while(sensor[0]==0){readSensor();}
  while(sensor[1]==0){readSensor();}
  while(sensor[2]==0){readSensor();}
  while(sensor[3]==0){readSensor();}
  while(sensor[4]==0){readSensor();} 
}

void standBy(){
  while (digitalRead(irPin) == HIGH){
   stopDrive(); 
  }
}

void defaults(){
  next = 1;
  countIntersec = 0;
  countLeftJunc = 0;
  countRightJunc = 0;
}

//Count intersection
int intersection(){
  readSensor();
  if(sensor[4]==0 && sensor[0]==0){
    countIntersec++;
    stopDrive();
  }
  return countIntersec;
}

//Count left junction
int leftJunction(){
  readSensor();
  if(sensor[0]==0 && sensor[1]==0 && sensor[3]==1 && sensor[4]==1){
    countLeftJunc++;
  }
  return countLeftJunc;
}

//Count right junction
int rightJunction(){
  readSensor();
  if(sensor[4]==0 && sensor[3]==0 && sensor[2]==0 && sensor[0]==1){
    countRightJunc++;
  }
  return countRightJunc;
}


void table_1(){  
  defaults();
  while(next){ 
    goDrive(); 
    if(intersection() == 1){ //Intersection 1, Start
    next=0;
    } 
  }  
  backward(100, 100);
  turnBack(100, 100); 
  
  defaults();
  while(next){       
    goDrive();
    if(rightJunction() == 1){ //Found right junction stright
    next=0; 
    }     
  }
  stright1(100, 100);
  goDrive();
  
  defaults();
  while(next){             
    irValue = digitalRead(irPin);
    if(irValue == HIGH){              
      goDrive();
      if(leftJunction() == 1){ //Found left junction turn left
      next=0; 
      }     
    }
    if(irValue == LOW){      
      stopDrive();
    }    
  }
  turnLeft(100, 0); 
  
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Finish intersection
    next=0; 
    }    
  }
      stopDrive();
      standBy();
      backward(100, 100);
      turnBack(100, 100);
   
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Found intersection turn right
    next=0; 
    }    
  }
  turnRight(0, 100);

  defaults();
  while(next){ 
    goDrive();
    if(leftJunction() == 1){ //Found left junction, stright
    next=0; 
    }    
  }
  stright2(100, 100);
  goDrive();
    
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Intersection finish, Stop
    next=0; 
    }    
  }
  resetFunc(); 
}

void table_2(){ 
  defaults();
  while(next){
    goDrive(); 
    if(intersection() == 1){ //Intersection 1, Start
    next=0; 
    }    
  } 
  backward(100, 100);
  turnBack(100, 100); 
  
  defaults();
  while(next){       
    goDrive();
    if(rightJunction() == 1){ //Found right junction stright
    next=0; 
    }     
  }
  stright1(100, 100); 
  goDrive();  
  
  defaults();
  while(next){ 
    goDrive();
    if(leftJunction() == 1){ //Found leftJunction stright
    next=0; 
    }    
  }
  stright2(100, 100);
  goDrive();
  
  defaults();
  while(next){ 
    irValue = digitalRead(irPin);
    if(irValue == HIGH){              
      goDrive();
      if(leftJunction() == 1){ //Found left junction turn left
      next=0; 
      }     
    }
    if(irValue == LOW){      
      stopDrive();
    }
  }
  turnLeft(100, 0);
  
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Finish intersection
    next=0; 
    }    
  }
      stopDrive();
      standBy();
      backward(100, 100);               
      turnBack(100, 100);
   
  defaults();
  while(next){ 
    goDrive();
    if(rightJunction() == 1){ //Found right junction turn right
    next=0; 
    }    
  }
  turnRight(0, 100);

  defaults();
  while(next){ 
    goDrive();
    if(rightJunction() == 1){ //Next right Junction stright
      next=0; 
      } 
  }          
  stright1(100, 100);
  goDrive();
  
  defaults();
  while(next){ 
    goDrive();
    if(leftJunction() == 1){ //Next left Junction stright
      next=0; 
      } 
  }          
  stright2(100, 100);
  goDrive();
  
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Intersection 7, Stop
    next=0; 
    }     
  }
  resetFunc();
}

void table_3(){   
  defaults();
  while(next){
    goDrive(); 
    if(intersection() == 1){ //Intersection 1, Start
    next=0; 
    }    
  } 
  backward(100, 100);
  turnBack(100, 100);
   
  defaults();
  while(next){       
    goDrive();
    if(rightJunction() == 1){ //Found right junction stright
    next=0; 
    }     
  }
  stright1(100, 100);
  goDrive(); 
  
  defaults();
  while(next){ 
    goDrive();
    if(leftJunction() == 1){ //Found leftJunction stright
    next=0; 
    }    
  }
  stright2(100, 100);
  goDrive(); 
    
  defaults();
  while(next){ 
    irValue = digitalRead(irPin);
    if(irValue == HIGH){              
      goDrive();
      if(leftJunction() == 1){ //Found left junction turn left
      next=0; 
      }     
    }
    if(irValue == LOW){      
      stopDrive();
    }
  }
  turnRight(0, 100);
  
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Finish intersection
    next=0; 
    }    
  }
      stopDrive();
      standBy();
      backward(100, 100);                
      turnBack(100, 100);
   
  defaults();
  while(next){ 
    goDrive();
    if(leftJunction() == 1){ //Found left junction turn left
    next=0; 
    }    
  }
  turnLeft(100, 0);

  defaults();
  while(next){       
    goDrive();
    if(rightJunction() == 1){ //Found right junction stright
    next=0; 
    }     
  }
  stright1(100, 100); 
  goDrive(); 
  
  defaults();
  while(next){ 
    goDrive();
    if(leftJunction() == 1){ //Found leftJunction stright
    next=0; 
    }    
  }
  stright2(100, 100);
  goDrive();
  
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Intersection 7, Stop
    next=0; 
    }    
  }
  resetFunc();
}

void goHome(){ 
  defaults();
  while(next){
    goDrive(); 
    if(intersection() == 1){ //Intersection 1, Start
    next=0; 
    }    
  }
  backward(100, 100);
  turnBack(100, 100); 
  
  defaults();
  while(next){       
    goDrive();
    if(rightJunction() == 1){ //Found right junction stright
    next=0; 
    }     
  }
  turnRight(0, 100);
  
  defaults();
  while(next){ 
    goDrive();
    if(intersection() == 1){ //Intersection 7, Stop
    next=0; 
    }    
  }
  //backward(100, 100);
  turnBack(100, 100);
  delay(1100); 
  backward(100, 100);
  delay(1000);
  endDrive();   
}

int readKeypad(){
  int value = analogRead(A1);
  Serial.println(value);
  if(value == 210){ //Button value 1
    mode = 1; //The robot moves to the first place
  }
  if(value == 152){ //Button value 2
    mode = 2; //The robot moves to the second place
  }
  if(value == 86){ //Button value 3
    mode = 3; //The robot moves to the third place
  }
  if(value == 6){ //Button value A
    mode  = 4; //Back to base
  }
    return mode;
}

void mission(){
  goDrive();
  if(sensor[4] == 0 && sensor[0] == 0){ 
  stopDrive(); 
  mode = readKeypad();

  switch(mode){
    case 1: 
      table_1();
      break;

    case 2:
      table_2();
      break;

    case 3:
      table_3();
      break;

    case 4:
      goHome();

    case 0:
      stopDrive();
      break;
    }
  }
}
