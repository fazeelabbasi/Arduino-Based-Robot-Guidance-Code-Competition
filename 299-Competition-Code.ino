#include <Servo.h>
#include <QSerial.h>

//Robot pin declarations
int rDir = 7;
int lDir = 4;
int rSpe = 6;
int lSpe = 5;
int leftWheelSpeed = 150;                             //set left wheel speed
int rightWheelSpeed = 250;                            //set right wheel speed
int IR = 2;                                          //used for acquiring initial position
int bumper = 3;                                      //both bumpers are wired to the same circuit
int leftSensor = A3;
int centerSensor = A4;
int rightSensor = A5;
int const THRESHOLD = 700;                           //threshold for light sensors
int const turnDelay = 1000;                          //time for half turn
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


//Configuring robot turning conditions
int P1[] = {6, 9, 4, 9, 6, 3, 9, 1, 3, 9, 6, 9, 2, 9, 5, 9, 2, 9, 2, 9, 2, 9}; //Position 1 [22]
int t1[] = {1, 2, 0, 2, 1, 0, 2, 1, 0, 2, 0, 2, 1, 2, 0, 2, 1, 2, 0, 2, 1, 2}; //0 is left turn, 1 is right turn, everything else is placeholder
int P2[] = {4, 9, 3, 9, 5, 9, 3, 9, 6, 1, 9, 1, 1, 9, 6, 2, 9, 1, 2, 9, 3, 9, 3, 9}; //Position 2 [24]
int t2[] = {1, 2, 0, 2, 1, 2, 0, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 1, 0, 2, 0, 2, 1, 2}; //0 is left turn, 1 is right turn, everything else is placeholder
int P3[] = {2, 9, 2, 9, 3, 9, 2, 9, 9, 9, 6, 1, 9, 1, 1, 9, 4, 9, 4, 9}; //Position 3 [20]
int t3[] = {1, 2, 0, 2, 1, 2, 0, 2, 2, 2, 0, 1, 2, 0, 1, 2, 0, 2, 1, 2}; //0 is left turn, 1 is right turn, everything else is placeholder
int intersections[25] = {};
int turns[25] = {}; 

//Other variables
int count = 0; //used to count intersections traversed
int action = 0; //used to traverse above arrays
boolean clawState = false; //used to determine if claw should be closed or open
int ball = 0; //denotes turn scenarios i.e. ball = 0 means going for first object, ball = 1 means going for second object, etc.
boolean getPosition = false; //used when acquiring initial position from IR signal


void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 25; i++){ //initialize arrays to -1, prevents excess turning when robot finishes task
    intersections[i] = -1;
    turns[i] = -1;
  }

//  while(!getPosition){ //robot will not run loop code until initial position has been acquired
    int signal = 48; //temporaily assign IR variable [ONLY FOR TESTING PURPOSES]
//    int signal = digitalRead(IR);
//    if (signal == LOW){
      checkPosition(signal);
//    }
//  }


  while(digitalRead(bumper) == LOW){
 
  }
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
//
//  //set up sensor
//  pinMode(leftSensor, INPUT);
//  pinMode(centerSensor, INPUT);
//  pinMode(rightSensor, INPUT);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

  driveForward();                  //drive forwards
  checkBumper();
  checkClaw();
  checkIntersection();
  checkEnd();  
}                    

//LOGIC FUNCTIONS TO EXECUTE DIRECTIONS
void checkClaw(){                               //checking claw state ([0] open or [1] closed)
  if (clawState == false){                         
      openGripper();     //keep claw open function
  }
  else if (clawState == true){
       grab();          //keep claw closed function
  }
}

boolean checkSensor(int sensor){
  if (analogRead(sensor) > THRESHOLD){
    return true;
  }
}


void checkIntersection(){                       //check if robot is passing intersection
  Serial.println("--------------------SENSOR START LCR------------------------");
  Serial.println(analogRead(leftSensor));
   Serial.println(analogRead(centerSensor));
    Serial.println(analogRead(rightSensor));
     Serial.println("--------------------SENSOR END LCR---------------------");
//  if ((checkSensor(analogRead(leftSensor)) == true )&&( checkSensor(analogRead(rightSensor)) == true)) {    //when robot passes intersection
   if ((analogRead(leftSensor) > THRESHOLD) && (analogRead(rightSensor) > THRESHOLD)) {
    count++;
    delay(1000);
  }
  Serial.println(count);
  checkTurn();                                  //only checks for a turn if intersection is passed
}

void checkTurn(){                               //check if robot needs to turn
  if (intersections[action] == count){         
    halfTurn(turns[action]);
    action++;
    count = 0;
  }
}

void checkBumper(){                            //check if robot impacts wall
  if (bumper == 1) {                         
     if (clawState == false){
        backUp();
        grab(); //close claw function
        clawState = !clawState;
     }    
     else if (clawState == true){
        openGripper();           //open claw function
        clawState = !clawState;
        ball++;
        backUp();
     }              
     count = 0;  
     turn180degrees();               //reverse and turn function
     action++;
  }  
}     

void checkEnd(){                              //check if robot is finished instructions
  if (ball == 5){
      stopDriving();           //robot stop function
  }
}

void checkPosition(int val){           //When IR signal is received, checks for position
  if (val == 48){                              //IR for position 1, 48 is ASCII for 0
    for(int i = 0; i < 22; i++){
       intersections[i] = P1[i];
       turns[i] = t1[i];
       getPosition = true; 
    }
  } else if (val == 49){                      //IR for position 2, 49 is ASCII for 1
    for(int i = 0; i < 24; i++){
       intersections[i] = P2[i];
       turns[i] = t2[i];
       getPosition = true;
    }
  } else if (val == 50){                      //IR for position 3, 50 is ASCII for 2
    for(int i = 0; i < 20; i++){
       intersections[i] = P3[i];
       turns[i] = t3[i];
       getPosition = true;
    }
  }
}

//DRIVE FUNCTIONS FOR ROBOT
void setSpeed(int leftSPEED, int rightSPEED) {
  if (leftSPEED < 0) {
    digitalWrite(lDir, LOW);
    leftSPEED = -leftSPEED;
  }
  else {
    digitalWrite(lDir, HIGH);
  }
  if (rightSPEED == 0){
    digitalWrite(rDir, LOW);
  }
  else {
    digitalWrite(rDir, HIGH);
  }

//  leftSPEED /= 2;
//  leftSPEED += 128;

  rightSPEED /= 2;
  
  if (rightSPEED > 0){
    rightSPEED += 128;
  }
  
  analogWrite(rSpe, rightSPEED);
  analogWrite(lSpe, leftSPEED);
  Serial.println("Left Speed:");
  Serial.println(leftSPEED);
  Serial.println("Right Speed:");
  Serial.println(rightSPEED);
}

void driveForward() {
  setSpeed(leftWheelSpeed, rightWheelSpeed);
//  if ((analogRead(rightSensor) > THRESHOLD) && (analogRead(centerSensor) < THRESHOLD) && (analogRead(leftSensor) < THRESHOLD))
//  {
//     Serial.println("Corecting LEFT BIAS");
//    analogWrite(rSpe, rightWheelSpeed - 15);
//    analogWrite(lSpe, leftWheelSpeed + 15);
//    delay(speedChangeDelay);
//  }
//  else if ((analogRead(leftSensor) > THRESHOLD) && (analogRead(centerSensor) < THRESHOLD) && (analogRead(rightSensor) < THRESHOLD))
//  {
//    Serial.println("Corecting RIGHT BIAS");
//    analogWrite(rSpe, rightWheelSpeed + 15);
//    analogWrite(lSpe, leftWheelSpeed - 15);
//    delay(speedChangeDelay);
//  }
}

void stopDriving(){
  setSpeed(0, 0);
}

void halfTurn(int dir){
  if(dir == 0){               //left turn
//     while(!checkSensor(leftSensor)){
      setSpeed(-leftWheelSpeed, rightWheelSpeed);
      delay(turnDelay);
//     }
  }
  else if (dir == 1){         //right turn
//     while(!checkSensor(rightSensor)){
      setSpeed(leftWheelSpeed, -rightWheelSpeed);
      delay(turnDelay);
//     }
  } 
}

void backUp(){
  int counter = 0;
  counter++;
  while(counter < 500){
  setSpeed(-leftWheelSpeed, -rightWheelSpeed);
  }
}

void turn180degrees(){
//   while(!checkSensor(centerSensor)){
    setSpeed(-leftWheelSpeed, rightWheelSpeed);
    delay(2*turnDelay);
//   } 
}

//GRIP FUNCTIONS FOR ROBOT
void openGripper(){
  tilt.write(releaseHeight);
  grip.write(releaseDice);
}

void grab(){
  tilt.write(grabHeight);
  grip.write(holdDice);
}
