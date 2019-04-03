#include <Servo.h>
#include <QSerial.h>

//Robot pin declarations
int rDir = 7;
int lDir = 4;
int rSpe = 6;
int lSpe = 5;
int adjSpeed = 60;
int lWSpeed = 90 + 30;                             //set left wheel speed
int rWSpeed = 96 + 30;                            //set right wheel speed
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
int holdDice = 90; //claw gripping state
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
  grip.write(90);
  tilt.write(straightUp);

  //set up motors
  pinMode(rDir, OUTPUT); 
  pinMode(lDir, OUTPUT); 
  pinMode(rSpe, OUTPUT); 
  pinMode(lSpe, OUTPUT);  
}

void loop() {    
  lineFollowInt(1);                 //drive forwards 
  turnRight();
  grab();
  lineFollowInt(1);
  turnLeft();
  release();
  lineFollowInt(2);                 //drive forwards 
  turnRight();
  grab();
  lineFollowInt(1);
  turnLeft();
  release();
//  lineFollowInt(4);                 //drive forwards 
//  turnRight();
  grab();
//  lineFollowInt(1);
//  turnLeft();
  release();
  lineFollowInt(4);                 //drive forwards 
  turnLeft();
  lineFollowInt(1);
  turnRight();
  grab();
  lineFollowInt(2);
  turnLeft();
  lineFollowInt(1);
  turnRight();
  release();
  lineFollowInt(3);
  turnLeft();
  grab();
  lineFollowInt(3);
  turnRight();
  release();
  while(true);
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
  delay(500);
  setSpeed(-lWSpeed, -rWSpeed);
  delay(225);
  analogWrite(rSpe, 0);
  analogWrite(lSpe, 0);
  delay(500);
  attachServo(true);
  tilt.write(straightUp);
  //delay(250);
   delay(25);
  pan.write(straightPan);
  delay(25);
  //delay(250);
  grip.write(0);
  delay(250);
  tilt.write(grabHeight);
  delay(1500);
  grip.write(90);
  delay(1000);
  tilt.write(straightUp);
  delay(250);
//  attachServo(false);
//  delay(1000);
//  setSpeed(-lWSpeed, -rWSpeed);
//  delay(350);
//  turnLeft();
//  turnLeft();
//  delay(300);
attachServo(false);
  delay(1000);
  setSpeed(-lWSpeed+10, -rWSpeed+10);
  while(!((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) > THRESHOLD))) {
//    if ((analogRead(rightSensor) > THRESHOLD) && (analogRead(leftSensor) < THRESHOLD)) {    //Auto-calibration
//      setSpeed(-lWSpeed, -(rWSpeed - adjSpeed-10));
//    }
//    else if ((analogRead(rightSensor) < THRESHOLD) && (analogRead(leftSensor) > THRESHOLD)) {
//      setSpeed(-(lWSpeed-adjSpeed-10), -rWSpeed);
//    }
//    else {
//      setSpeed(-lWSpeed+10, -rWSpeed+10);
//    }
  }
  turnLeft();
  delay(300);
}


void release()
{
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
      }}
  attachServo(true);
  tilt.write(180);
  delay(250);
  pan.write(90);
  delay(250);
  grip.write(90);
  delay(250);
  tilt.write(grabHeight);
  delay(50);
  grip.write(0);
  delay(250);
  tilt.write(180);
  delay(250);
   attachServo(false);
  delay(1000);
  setSpeed(-lWSpeed, -rWSpeed);
  delay(250);
  turnLeft();
  
  delay(300);
//   lWSpeed += 10;                             
//  rWSpeed += 10;        
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

<<<<<<< HEAD
pathCentre() {
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
  while(true);
=======
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
  release();

  //Third Object
  grab();
  release();

  //Fourth Object
  lineFollowInt(2);
  turnRight();
  grab();
  lineFollowInt(3);
  turnLeft();
  release();

  //Fifth Object
  lineFollowInt(4);
  turnRight();
  grab();
  lineFollowInt(3);
  turnLeft();
  release();
>>>>>>> ea8106a39df60263d2636d06bc2ef2de5849eca3
}
