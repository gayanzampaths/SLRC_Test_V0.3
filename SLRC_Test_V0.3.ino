#include <QTRSensors.h>
#include <Encoder.h>

//moter drive digital pins
#define In1 4  //Left
#define In2 5
#define In3 6  //Right
#define In4 7

//moter drive analog pins
#define ENR 8
#define ENL 3

//PID CONTROL LOOP
#define Kp 0.08         // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.12          // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
#define rightMaxSpeed 250   // max speed of the robot
#define leftMaxSpeed 250    // max speed of the robot
#define rightBaseSpeed 200    // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 200   // this is the speed at which the motors should spin when the robot is perfectly on the line

// INFRARED PANEL
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500    // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   A7    // emitter is controlled by digital pin 2
#define IR_TRIGGER_LIMIT 350  // infraRed panel trigger point

// buzzer pin
#define BUZZER      11    

//Encoder
Encoder leftEn(19, 18);     //Encoder of Left Motor
Encoder rightEn(20, 21);      //Encoder of Right Motor

//QRT IR Sensor panel
QTRSensorsRC FrontIrPanel((unsigned char[]) {A8, A9, A10, A11, A12, A13, A14, A15},  NUM_SENSORS, TIMEOUT, EMITTER_PIN);

//Variables
int position;                   //centroid value of the line
unsigned int sensorValues[NUM_SENSORS];       //stores the sensor readings
int lastError = 0;                  //used to PID derivative calculation
char old;
char ne;
char maze_at_node[100];               //used in maze solving to store maze
byte maze_node_no=0;                //used in maze solving to determine step number

void setup() {
  // put your setup code here, to run once:
  //initializing all
  initAll();
  //start functional
  lineFollow();
  maze();
  goSteps(2200,2200,100);
  hardStop();
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

void initAll(){
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);
  pinMode(In3,OUTPUT);
  pinMode(In4,OUTPUT);
  pinMode(ENL,OUTPUT);
  pinMode(ENR,OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);

  goSteps(2000,2000,100);
  autoCalibration();
}

//Auto calibrate QRT IR Sensor panel
void autoCalibration(){
  pinMode(13, HIGH);
  digitalWrite(13, HIGH);
  
  analogWrite(ENL, 50);
  analogWrite(ENR, 50);
  digitalWrite(In1, 1);
  digitalWrite(In2, 0);
  digitalWrite(In3, 0);
  digitalWrite(In4, 1);

  for (int i = 0; i < 60; i++) {
    FrontIrPanel.calibrate();    
  }

  analogWrite(ENL, 50);
  analogWrite(ENR, 50);
  digitalWrite(In1, 0);
  digitalWrite(In2, 1);
  digitalWrite(In3, 1);
  digitalWrite(In4, 0);

  for (int i = 0; i < 100; i++) {
    FrontIrPanel.calibrate();
  }
  pinMode(13, HIGH);
  digitalWrite(13, HIGH);

  analogWrite(ENL, 50);
  analogWrite(ENR, 50);
  digitalWrite(In1, 1);
  digitalWrite(In2, 0);
  digitalWrite(In3, 0);
  digitalWrite(In4, 1);

  for (int i = 0; i < 60; i++) {
    FrontIrPanel.calibrate();    
  }
  stopNow();
  digitalWrite(13, LOW);
}

//QRT IR Sensor front IR array
byte FrontIrArray() {                                //Required Part
  byte x = 0;
  position = FrontIrPanel.readLine(sensorValues);
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > IR_TRIGGER_LIMIT) {
      bitWrite(x, i, 1);
      Serial.print("1");
    } else {
      Serial.print("0");
    }
  }
  Serial.println();
  return x;
}

//PID LINE FOLLOWER
void lineFollow() {
 byte c=FrontIrArray();
  while(c!=B00000000 && c!=B11111111 && c!=B01111111 && c!=B00111111 && c!=B11111110 && c!=B11111100 && c!=B00011111 && c!=B11111000){
  
    unsigned int sensors[8];
    int pos = FrontIrPanel.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
    int error = pos - 3500;
    int rightMotorSpeed;
    int leftMotorSpeed;
  
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    //Serial.println(motorSpeed);
    if (motorSpeed < 0) {
      rightMotorSpeed = rightBaseSpeed - abs(motorSpeed);
      leftMotorSpeed = leftBaseSpeed;
    } else {
      rightMotorSpeed = rightBaseSpeed;
      leftMotorSpeed = leftBaseSpeed - abs(motorSpeed);
    }
  
    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
    if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
    //Serial.print(leftMotorSpeed); Serial.print(",\t"); Serial.println(rightMotorSpeed);
  
    analogWrite(ENL, leftMotorSpeed);
    analogWrite(ENR, rightMotorSpeed);
    digitalWrite(In1, 1);
    digitalWrite(In2, 0);
    digitalWrite(In3, 1);
    digitalWrite(In4, 0);
  
     c=FrontIrArray();
  }

}

//Movement Functions
void forward() {
  analogWrite(ENL, leftBaseSpeed);
  analogWrite(ENR, rightBaseSpeed);
  digitalWrite(In1, 1);
  digitalWrite(In2, 0);
  digitalWrite(In3, 1);
  digitalWrite(In4, 0);

}

void left() {
  analogWrite(ENL, leftBaseSpeed);
  analogWrite(ENR, rightBaseSpeed);
  digitalWrite(In1, 0);
  digitalWrite(In2, 1);
  digitalWrite(In3, 1);
  digitalWrite(In4, 0);

}

void right() {
  analogWrite(ENL, leftBaseSpeed);
  analogWrite(ENR, rightBaseSpeed);
  digitalWrite(In1, 1);
  digitalWrite(In2, 0);
  digitalWrite(In3, 0);
  digitalWrite(In4, 1);

}

void left90c(){          // Select left hand side
  //goSteps(2200,2200,100);

  leftEn.write(0);
  rightEn.write(0);

  analogWrite(ENR, 100);
  analogWrite(ENL, 100);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  
  while((leftEn.read() >= -2300) && (rightEn.read() <= 2300)){  //rough movement
    if(leftEn.read() <= -2300){
      analogWrite(ENL, 0);
      digitalWrite(In3, LOW);
    }
    if(rightEn.read() >= 2300){
      analogWrite(ENR, 0);
      digitalWrite(In2, LOW);
    }
  }

  analogWrite(ENR, 100);
  analogWrite(ENL, 100);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);

  while(1){                         //fine tune to center
    byte ir=FrontIrArray();
    if(ir==B00111100 || ir==B00111000 || ir==B00011100 || ir==B00011000 || ir==B00010000 || ir==B00001000){
      break;
    }
    delay(5);
  }
  analogWrite(ENR, 0);
  analogWrite(ENL, 0);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
}

void right90c(){          // Select right hand side
  //goSteps(2200,2200,100);

  leftEn.write(0);
  rightEn.write(0);

  analogWrite(ENR, 100);
  analogWrite(ENL, 100);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);

  while((leftEn.read() <= 2300) && (rightEn.read() >= -2300)){  //rough movement
    if(leftEn.read() >= 2300){
      analogWrite(ENL, 0);
      digitalWrite(In4, LOW);
    }
    if(rightEn.read() <= -2300){
      analogWrite(ENR, 0);
      digitalWrite(In1, LOW);
    }
  }

  analogWrite(ENR, 100);
  analogWrite(ENL, 100);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);

  while(1){                         //fine tune to center
    byte ir=FrontIrArray();
    if(ir==B00111100 || ir==B00111000 || ir==B00011100 || ir==B00011000 || ir==B00010000 || ir==B00001000){
      break;
    }
    delay(5);
  }
  analogWrite(ENR, 0);
  analogWrite(ENL, 0);
  digitalWrite(In1, LOW);
  digitalWrite(In4, LOW);
  
}

void uTurn() {
  leftEn.write(0);
  rightEn.write(0);

  analogWrite(ENL, 180);
  analogWrite(ENR, 180);
  digitalWrite(In1, 1);
  digitalWrite(In2, 0);
  digitalWrite(In3, 0);
  digitalWrite(In4, 1);

  while ((leftEn.read() <= 6500) && (rightEn.read() >= -6500)) {
    if (leftEn.read() >= 6500)  analogWrite(ENL, 0);
    if (rightEn.read() <= -6500)  analogWrite(ENR, 0);
  }

  analogWrite(ENL, 0);
  analogWrite(ENR, 0);
  left();
  delay(50);
  stopNow();
}

void reverse() {
  analogWrite(ENL, leftBaseSpeed);
  analogWrite(ENR, rightBaseSpeed);
  digitalWrite(In1, 0);
  digitalWrite(In2, 1);
  digitalWrite(In3, 0);
  digitalWrite(In4, 1);

}

void stopNow() {
  analogWrite(ENL, 0);
  analogWrite(ENR, 0);
  digitalWrite(In1, 0);
  digitalWrite(In2, 0);
  digitalWrite(In3, 0);
  digitalWrite(In4, 0);
}

//Hard Stop
void hardStop(){
  reverse();
  delay(50);
  stopNow();  
}

void goto_fwd(){          // Select forward
  goSteps(1100,1100,100);
}

//Go Steps
void goSteps(long l, long r,  byte spd){
  
stopNow();
  
  leftEn.write(0);
  rightEn.write(0);
  
  if(l > 0) {
    digitalWrite(In1, 1);
    digitalWrite(In2, 0);
    analogWrite(ENL, spd);

  }
  else{ 
    digitalWrite(In1, 0);
    digitalWrite(In2, 1);
    analogWrite(ENL, spd);
  }  
    if(r > 0) {
    digitalWrite(In3, 1);
    digitalWrite(In4, 0);
    analogWrite(ENR, spd);

  }
  else{ 
    digitalWrite(In3, 0);
    digitalWrite(In4, 1);
    analogWrite(ENR, spd);
  } 
  
  boolean lm = false;
  boolean rm = false;
  while(true){
    
    if(abs(rightEn.read()) >= abs(r)){
    digitalWrite(In3, 0);
    digitalWrite(In4, 0);
    analogWrite(ENR, 0);
      rightEn.write(0);
      rm = true;
      if(lm) break;
    }
    
    if(abs(leftEn.read()) >= abs(l)){
    digitalWrite(In1, 0);
    digitalWrite(In2, 0);
    analogWrite(ENL, 0);
      leftEn.write(0);
      lm = true;
      if(rm) break;
    }
  }
  hardStop();
}

//MAZE SOLVIGN IMPLIMENTATION
char feature_detect(){  // This Switch must be improved to avoid errors.......................................................
  byte signal=FrontIrArray();
  switch(signal){
    case B00000000:return 'W';
    case B11111111:return 'B';
    
    
    case B00000111:
    case B00001111:
    case B00011111:
    case B00111111:
    case B01111111:return 'R';
    

    case B11100000:
    case B11110000:
    case B11111000:
    case B11111100:
    case B11111110:return 'L';
    default: return 'F';    
  }
}

void maze(){
  while(1){
    Serial.println(feature_detect());
    old = feature_detect();
    if(old=='L'){
      tone(BUZZER,3000,500);
      goSteps(900,900,50);
      ne = feature_detect();
      Serial.println(ne);
      if(ne=='F'){
        Serial.println('l');
        maze_at_node[maze_node_no]='S';
        maze_node_no++;
        goto_fwd();
      }
      if(ne=='W'){
        Serial.println('L');
        maze_at_node[maze_node_no]='L';
        maze_node_no++;
        left90c();
      }
      lineFollow();
      continue;
    }

    if(old=='R'){
      tone(BUZZER,3000,500);
      goSteps(900,900,50);
      ne = feature_detect();
      Serial.println(ne);
      if(ne=='F'){
        Serial.println('r');
        maze_at_node[maze_node_no]='R';
        maze_node_no++;
        right90c();
      }
      if(ne=='W'){
        Serial.println('R');
        maze_at_node[maze_node_no]='R';
        maze_node_no++;
        right90c();
      }
      lineFollow();
      continue;
    }

    if(old=='B'){
      tone(BUZZER,3000,500);
      goSteps(900,900,50);
      ne = feature_detect();
      Serial.println(ne);
      if(ne=='F'){
        Serial.println('+');
        maze_at_node[maze_node_no]='R';
        maze_node_no++;
        right90c();
      }
      if(ne=='W'){
        Serial.println('T');
        maze_at_node[maze_node_no]='R';
        maze_node_no++;
        right90c();
      }
      if(ne=='B'){
        Serial.println('E');
        maze_at_node[maze_node_no]='E';
        maze_node_no++;
        tone(BUZZER,3000,2000);
        break;
      }
      lineFollow();
      continue;
    }

    if(old=='W'){
      tone(BUZZER,3000,500);
      goSteps(900,900,50);
      ne = feature_detect();
      Serial.println(ne);
      if(ne=='W'){
        Serial.println('D');
        maze_at_node[maze_node_no]='U';
        maze_node_no++;
        uTurn();
      }
      lineFollow();
      continue;
    }
  }
}

