#include <Servo.h>
#include <QSerial.h>

//Robot pin declarations
int rDir = 7;
int lDir = 4;
int rSpe = 6;
int lSpe = 5;
int adjSpeed = 60;
int lWSpeed = 90 + 40;                             //set left wheel speed
int rWSpeed = 96 + 40;                            //set right wheel speed
int IR = 2;                                          //used for acquiring initial position
int bumper = 2;                                      //both bumpers are wired to the same circuit
int leftSensor = A0;
int rightSensor = A1;
int centerSensor = A2;
int const THRESHOLD = 550;                           //threshold for light sensors
int const turnDelay = 1200-600;                          //time for half turn
int const speedChangeDelay = 200;

Servo grip, tilt, pan;
int panPin = 8;
int gripPin = 10;
int tiltPin = 9;
int grabHeight = 75; //height for gripping
int releaseHeight = 120; //height for releasing object
int straightPan = 90; //angle of claw (facing forward)
int straightUp = 180;

//open 0 - 180 close
int holdDice = 120; //claw gripping state
int releaseDice = 5; //claw open state

//Other variables

boolean clawState = false; //used to determine if claw should be closed or open

void setup() { 
  Serial.begin(9600);

  //set up claw
  grip.attach(gripPin);
  tilt.attach(tiltPin);
  pan.attach(panPin);
  pan.write(90);
  grip.write(0);
  tilt.write(straightUp);

  //set up motors
  pinMode(rDir, OUTPUT); 
  pinMode(lDir, OUTPUT); 
  pinMode(rSpe, OUTPUT); 
  pinMode(lSpe, OUTPUT);
  waitButton();
}

void loop() {    
//  if()pathLeft();
//    else if()pathCentre();
//      else if()
//        pathRight();
        pathCentre();
//  pathLeft();
        while(1);
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
    while(1){
     
      if((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) < THRESHOLD)){      //Auto-calibration
        setSpeed(lWSpeed, (rWSpeed - adjSpeed));
        //delay(100);
      }
      else if((analogRead(rightSensor) < THRESHOLD) && (analogRead(leftSensor) > THRESHOLD)){
        setSpeed((lWSpeed - adjSpeed), rWSpeed);
       // delay(100);
      }
      else{
        setSpeed(lWSpeed,rWSpeed);
      }

      if((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) > THRESHOLD)){
        while((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) > THRESHOLD));
        delay(175);
        break;
      }
    }
    Serial.println("Intersection passed!");
  }
  setSpeed(0,0);
}

void turnLeft()
{
  delay(250);
  Serial.println("LEFT TURN");
  setSpeed(-(lWSpeed-15),rWSpeed-15);
  delay(turnDelay);
  while (analogRead(centerSensor) < THRESHOLD) {}
  setSpeed(0,0);
}

void turnRight(){
  delay(250);
  Serial.println("RIGHT TURN");
  setSpeed(lWSpeed-15,-(rWSpeed-15));
  delay(turnDelay);
  while (analogRead(centerSensor) < THRESHOLD) {}
  setSpeed(0,0);
}

void grab()
{
  lWSpeed += 30;
  rWSpeed += 30;
  tilt.write(straightUp);
  Serial.println("HITING TO GRAB");
  setSpeed(lWSpeed, rWSpeed);
  while (digitalRead(bumper) == 1) {
    if ((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) < THRESHOLD)) {    //Auto-calibration
      setSpeed(lWSpeed, rWSpeed - adjSpeed);
    }
    else if ((analogRead(rightSensor) < THRESHOLD) && (analogRead(leftSensor) > THRESHOLD)) {
      setSpeed(lWSpeed-adjSpeed, rWSpeed);
    }
    else {
      setSpeed(lWSpeed, rWSpeed);
    }
  }
  Serial.println("GRABBING");
  delay(50);
  lWSpeed -= 30;
  rWSpeed -= 30;
  setSpeed(-lWSpeed, -rWSpeed);
  delay(200);
  analogWrite(rSpe, 0);
  analogWrite(lSpe, 0);
  delay(100);
  attachServo(true);
  tilt.write(straightUp);
  //delay(250);
   delay(25);
  pan.write(straightPan);
  delay(25);
  grip.write(0);
  tilt.write(grabHeight);
  delay(1750);
  grip.write(holdDice);
  delay(700);
  tilt.write(straightUp);
  delay(250);
  attachServo(false);
  delay(100);
  setSpeed(-lWSpeed+10, -rWSpeed+10);
  while(!((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) > THRESHOLD)));
  setSpeed(40,40);
  turnLeft();
  delay(150);
}


void release()
{
  lWSpeed += 40;
  rWSpeed += 40;
  tilt.write(straightUp);
  setSpeed(lWSpeed, rWSpeed);
  while (digitalRead(bumper) == 1) {
    
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
  }
  attachServo(true);
  tilt.write(180);
  delay(25);
  pan.write(90);
  delay(25);
  grip.write(holdDice);
  delay(25);
  tilt.write(grabHeight);
  delay(100);
  grip.write(0);
  delay(250);
  tilt.write(180);
  delay(250);
  attachServo(false);
  delay(100);
  lWSpeed -= 30;
  rWSpeed -= 30;
  setSpeed(-lWSpeed, -rWSpeed);
  delay(250);
  turnLeft();
  
  delay(300);      
}

void attachServo(bool tf)
{
  if (tf == true)
  {
    grip.attach(gripPin);
    tilt.attach(tiltPin);
    pan.attach(panPin);
  }
  else
  {
    grip.detach();
    tilt.detach();
    pan.detach();
  }
}
void waitButton()
{
  while (digitalRead(bumper) == HIGH) {}
  while (digitalRead(bumper) == LOW) {}
  delay(500);
}

void pathCentre() {
  lineFollowInt(3);                 //drive forwards 
  turnRight();
  grab();
  lineFollowInt(2);
  turnLeft();
  release();
  
  lineFollowInt(3);                 //drive forwards 
  turnLeft();
  grab();
  lineFollowInt(2);
  turnRight();
  release();

  lWSpeed += 5;
  rWSpeed += 5;

  grab();
  release();
  
  lineFollowInt(5);                 //drive forwards 
  turnLeft();
  lineFollowInt(2);
  turnRight();
  grab();
  lineFollowInt(1);
  turnLeft();
  lineFollowInt(2);
  turnRight();
  release();

  lWSpeed += 5;
  rWSpeed += 5;
  
  lineFollowInt(5);                 //drive forwards 
  turnRight();
  lineFollowInt(2);
  turnLeft();
  grab();
  lineFollowInt(1);
  turnRight();
  lineFollowInt(2);
  turnLeft();
  release();
}
void pathLeft(){
  //First Object
  lineFollowInt(5);
  turnRight();
  grab();
  lineFollowInt(3);
  turnLeft();
  release();

  //Second Object
  lineFollowInt(1);
  turnLeft();
  grab();
  lineFollowInt(1);
  turnRight();
  release();

  lWSpeed += 5;
  rWSpeed += 5;

  //Third Object
  grab();
  release();

  //Fourth Object
  lineFollowInt(2);
  turnLeft();
  grab();
  lineFollowInt(1);
  turnRight();
  release();
  
  lWSpeed += 5;
  rWSpeed += 5;

  //Fifth Object
  lineFollowInt(4);
  turnLeft();
  grab();
  lineFollowInt(1);
  turnRight();
  release();
}
void pathRight(){
  //First Object

  lineFollowInt(1);
  turnRight();
  grab();
  lineFollowInt(1);
  turnLeft();
  release();
  
  lineFollowInt(5);
  turnLeft();
  grab();
  lineFollowInt(3);
  turnRight();
  release();

  lWSpeed += 5;
  rWSpeed += 5;

  grab();
  release();

  lineFollowInt(2);
  turnRight();
  grab();
  lineFollowInt(1);
   turnLeft();
  release();

  lWSpeed += 5;
  rWSpeed += 5;

  lineFollowInt(4);
  turnRight();
  grab();
  lineFollowInt(1);
  turnLeft();
  release();
  }
