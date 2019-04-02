#include <Servo.h>
#include <QSerial.h>

//Robot pin declarations
int rDir = 7;
int lDir = 4;
int rSpe = 6;
int lSpe = 5;
int adjSpeed = 15;
int lWSpeed = 90;                             //set left wheel speed
int rWSpeed = 100;                            //set right wheel speed
int IR = 2;                                          //used for acquiring initial position
int bumper = 2;                                      //both bumpers are wired to the same circuit
int leftSensor = A0;
int rightSensor = A1;
int centerSensor = A2;
int const THRESHOLD = 630;                           //threshold for light sensors
int const turnDelay = 1200;                          //time for half turn
int const speedChangeDelay = 200;

Servo grip, tilt, pan;
int panPin = 8;
int gripPin = 10;
int tiltPin = 9;
int grabHeight = 90; //height for gripping
int releaseHeight = 120; //height for releasing object
int straightPan = 90; //angle of claw (facing forward)

//open 0 - 180 close
int holdDice = 120; //claw gripping state
int releaseDice = 40; //claw open state





//Other variables

boolean clawState = false; //used to determine if claw should be closed or open



void setup() {

 
  Serial.begin(9600);

  //set up claw
  grip.attach(gripPin);
  tilt.attach(tiltPin);
  pan.attach(panPin);
  pan.write(90);

  //set up motors
  pinMode(rDir, OUTPUT); 
  pinMode(lDir, OUTPUT); 
  pinMode(rSpe, OUTPUT); 
  pinMode(lSpe, OUTPUT);

  
}

void loop() {
 
     
  lineFollowInt(1);                 //drive forwards 
  turnLeft();

//  setSpeed(-lWSpeed, -rWSpeed);

}                    

//LOGIC FUNCTIONS TO EXECUTE DIRECTIONS


//DRIVE FUNCTIONS FOR ROBOT
void setSpeed(int leftSPEED, int rightSPEED) {
  if (leftSPEED < 0) {
    digitalWrite(lDir, LOW);
    leftSPEED = -leftSPEED;
  }
  else digitalWrite(lDir, HIGH);
  if (rightSPEED < 0) {
    digitalWrite(rDir, LOW);
    rightSPEED = -rightSPEED;
  }
  else digitalWrite(rDir, HIGH);

  analogWrite(rSpe, rightSPEED);
  analogWrite(lSpe, leftSPEED);

  Serial.println("Left Speed:");
  Serial.println(leftSPEED);
  Serial.println("Right Speed:");
  Serial.println(rightSPEED);
}

void lineFollowInt(int i){
  for(int n = 0; n < i; n++){
    setSpeed(lWSpeed,rWSpeed);
          Serial.println("--------------------N------------------------");
    Serial.println(n);
          Serial.println("--------------------N------------------------");

    while(1){
      
//      Serial.println("--------------------SENSOR START LCR------------------------");
//      Serial.println(analogRead(leftSensor));
//      Serial.println(analogRead(centerSensor));
//      Serial.println(analogRead(rightSensor));
//      Serial.println("--------------------SENSOR END LCR------------------------");
     
      if((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) < THRESHOLD)){      //Auto-calibration
        setSpeed(lWSpeed, (rWSpeed - adjSpeed));
        delay(100);
      }
      else if((analogRead(rightSensor) < THRESHOLD) && (analogRead(leftSensor) > THRESHOLD)){
        setSpeed((lWSpeed - adjSpeed), rWSpeed);
        delay(100);
      }
      else{
        setSpeed(lWSpeed,rWSpeed);
      }

      if((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) > THRESHOLD)){
        while((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) > THRESHOLD));
        delay(250);
        break;
      }
    }
    Serial.println("Intersection passed!");
  }
//  setSpeed(0,0);
}


void stop(){

  while(1){
     setSpeed(0,0);
    }
  }

void turnLeft()
{
  delay(500);
  Serial.println("LEFT TURN");
  setSpeed(-lWSpeed,rWSpeed);
  delay(turnDelay);
  while (analogRead(centerSensor) < THRESHOLD) {}
  analogWrite(rSpe, 0);
  analogWrite(lSpe, 0);
}

void turnR(int i){
  int flag = 0;
  for(int n = 0; n < i; n++){
    setSpeed(lWSpeed, -rWSpeed);
    delay(turnDelay/2);
    while(flag < 15){
      if((analogRead(rightSensor) > THRESHOLD) || (analogRead(centerSensor) > THRESHOLD)){
        flag++; 
      }
    }
    flag = 0;
  }
//  setSpeed(-lWSpeed, rWSpeed);
//  delay(250);
//  setSpeed(0, 0);
}

void turn180(int i){
  int flag = 0;
  for(int n = 0; n < i; n++){
    setSpeed(-lWSpeed, rWSpeed);
    delay(turnDelay);
    while(flag < 25){
      if((analogRead(leftSensor) > THRESHOLD) || (analogRead(centerSensor) > THRESHOLD)){
        flag++; 
      }
    }
    flag = 0;
  }
}
