/********************************************************
  Autonomous Loader Code

  Daniel Dubinko 2022/11/27 

*********************************************************/

#include <Servo.h>
Servo leftWheel, rightWheel;
Servo myServoA, myServoB;

// -----------Variable Initialization--------------
// Pin Assignments
const int RED = 10;    // red LED Pin
const int GRN = 9;     // green LED Pin
const int YLW = 5;     // yellow LED Pin
const int BUTTON = 7;  // pushbutton Pin
const int SHARP = A3;  // sharp Sensor on Analog Pin 3 (distance sensor)

int lineSensor = 0;
int servoPinA = 11;    // left Bucket Servomotor
int servoPinB = 12;    // right Bucket Servomotor

// Curl Servo Set-Up Angles
int myAngleA1 = 151;   // initial angle, bucket lifts off ground if too high
int posA = myAngleA1;  // if set to 180, bucket lifts robot off of ground
int myAngleA2 = 100;   // highest angle (lift), puts almost straight, set to 110

int myAngleB1 = 90;    // higher angle
int posB = myAngleB1;
int myAngleB2 = 70;    //lower angle
int myAngleUnLevel = 150; // drop off material angle

int loopCounter = 0;   // pick up/ drop off counter
int intersectionCounter = 0; //intersection counter
int sharpCounter = 0;  //sharpsensor counter to account for ghost sensing

const int THRESH = 1480; // < threshold sensor sees white and > threshhold sensor sees black
int MOTOR_R = 3;         // right motor signal pin
int MOTOR_L = 4;         // left motor signal pin
const int stopPulse = 148;    // stop speed for motors

// Delta = speed above (+) or below (-) stop speed (must be positive)
const float delta = 10;
const float offset = 1;
int del = 0;


const int LSENSOR = A2;  // left Sensor on Analog Pin 1
const int RSENSOR = A1;  // right Sensor on Analog Pin 2

//global variables
int lvalue = 0;    //left sensor value
int rvalue = 0;    //right sensor value

int value = 0;
int mv_value = 0;

unsigned long previousMillis;
const long interval = 500;



// -----------Set-up Routine--------------
void setup() {

  // Initialize led pins as outputs.
  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);

  // Initialize button pins as inputs
  pinMode(BUTTON, INPUT);
  pinMode(SHARP, INPUT);

  // Set motor control pins as servo pins
  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);

  // Initialize line following sensor pins as inputs
  pinMode(LSENSOR, INPUT);
  pinMode(RSENSOR, INPUT);

  // Set-Up Lift and Curl Motors
  myServoA.write(posA);       // Servo A starting position
  myServoA.attach(servoPinA); // Attaches the servo to the servo object
  myServoB.write(posB);       // Servo B starting position
  myServoB.attach(servoPinB); // Attaches the servo to the servo object

  // Power-on loop, program runs after button is pressed
  do {
    runMotors(0, 0);
    toggleLED(GRN);      // motors stopped, green LED flashing
  } while (digitalRead(BUTTON) == LOW);
  delay(500); // time to clear button
}


//-------------Main Routine ----------------
void loop() {

  //Sharp Sensor Reading
  value = analogRead(SHARP); // ADC Output
  mv_value = map(value, 0, 1023, 0, 3300); //convert AtoD count to millivolts
  updateSensors();

  if (mv_value >= 1650) { 
    sharpCounter += 1;
    delay(40);
    
    if (sharpCounter == 3) { //runs check and stops for 100 milliseconds
      sharpCounter = 0;
      runMotors(0, 0);
      delay(100);
      loopCounter += 1; // adds one to loop to indicate drop off or pick up
      
      if (loopCounter % 2 != 0) { // count ==  odd, MiniBot is picking up material
        turn180(delta);
        pickUp();

      }
      else if (loopCounter % 2 == 0) { // count ==  even, MiniBot is droping material
        turn180(delta);
        dropOff();
      }
    }

  }
  else if (lvalue > THRESH && rvalue > THRESH) { //both sensors see black, intersection was hit

    intersection(9, 6);
    intersectionCounter += 1;
  }
  else {
    if (intersectionCounter % 2 != 0) { // intersectionCoutner ==  odd, MiniBot drives slower
      intersectionDriving(); //drives slower when pickup up and dropping off material
    }
    else if (intersectionCounter % 2 == 0) { // intersectionCoutner ==  even, MiniBot drives faster
      straightPath(); //drives faster when carrying material to next location
    }
  }

}

//********** Functions (subroutines) ******************

// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}
// Toggle an LED on/off
void toggleLED(int colour) {
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(colour, HIGH);
  delay(250);
  digitalWrite(colour, LOW);
  delay(250);
}

// Wheel function
void runMotors(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL) * 10;  // length of pulse in microseconds
  int pulseR = (stopPulse + deltaR) * 10;
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR);
}

// Turn Rover Clockwise
void turnRover(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL) * 10;  // length of pulse in microseconds
  int pulseR = (stopPulse - deltaR) * 10;  //left wheel turns forward and right wheel turns back
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR);
}

//Update sensors 
void updateSensors() {
  //read the sensor value
  lvalue = analogRead(LSENSOR);
  rvalue = analogRead(RSENSOR);

  //map the values into millivolts (assuming 3000 mV reference voltage)
  lvalue = map(lvalue, 0, 1023, 0, 3000);
  rvalue = map(rvalue, 0, 1023, 0, 3000);

}

// Intersection driving code
void intersectionDriving() {
  if (rvalue < THRESH && lvalue < THRESH) { // checks if both left and right sensor off of line

    turnOnLED(GRN);
    runMotors(10, 10 - 1); //10 works _ - 1 on the right side
    runMotors(5, 5 - 1); //4 works

  }
  else if (lvalue > THRESH) { // check if left sensor is on line //
    turnOnLED(YLW);
    runMotors(3 , 10 - 1); //

  }
  else if (rvalue > THRESH) { // check if right sensor is on line //
    turnOnLED(RED);
    runMotors(10 , 3 - 1); //4, 10 works
  }
}
// Straight path driving code
void straightPath() {
  if (rvalue < THRESH && lvalue < THRESH) { // checks if both left and right sensor off of line

    turnOnLED(GRN);
    runMotors(10, 10 - 1.5); //10 works _ - 1 on the right side
    runMotors(6, 6 - 1.5); //4 works

  }
  else if (lvalue > THRESH) { // check if left sensor is on line //
    turnOnLED(YLW);
    runMotors(3, 11 - 1); //

  }
  else if (rvalue > THRESH) { // check if right sensor is on line //
    turnOnLED(RED);
    runMotors(11 , 3 - 0.5); //4, 10 works
  }
}

// Correct Minibot wheels until aligned
void fixWheels() {
  while (1) {
    updateSensors();
    if (lvalue < THRESH && rvalue < THRESH) {
      break;
    }
    else if (lvalue > THRESH) { // check if left sensor is on line //
      turnOnLED(YLW);
      runMotors(0 , 7);
      delay(300);
      runMotors(7 , 0);
      delay(300);
      runMotors(0 , -14 + 1 );
      delay(300);
      runMotors(-14, 0);
      delay(300);

    }
    else if (rvalue > THRESH) { // check if right sensor is on line //
      turnOnLED(RED);
      runMotors(7 , 0);
      delay(300);
      runMotors(0 , 7 - 0.5 );
      delay(200);
      runMotors(-14 , 0);
      delay(300);
      runMotors(0 , -14 + 0.5 );
      delay(200);
    }
  }

}

//Intersection 90 degree turn
void intersection(int deltaL, int deltaR) { 

  runMotors(0, 0);
  delay(2000);
  runMotors(-10, -10 - 0.5);
  delay(750);
  turnRover(deltaL, deltaR);
  delay(1200);

}


// Lift and Curl servo movement 
void liftBucket()//lifting the Bucket
{
  delay (1000); // A couple seconds to stand back
  for (posA = myAngleA1; posA >= myAngleA2; posA--) {
    // Lift action
    myServoA.write(posA);
    delay(35);
  }
}

void dropBucket()//dropping the Bucket
{
  delay(1000);
  for (posA = myAngleA2; posA <= myAngleA1; posA++)
  { // Drop action
    myServoA.write(posA);
    delay(35);
  }
}

void levelBucket()//leveling the Bucket to keep material from falling
{
  delay (2000);
  for (posB = myAngleB1; posB >= myAngleB2; posB--) {
    // Scoop lower (level scooper)
    myServoB.write(posB);
    delay(20);
  }
}

void unlevelBucket() //unleveling the Bucket to drop material
{
  delay(500);
  for (posB = myAngleB2; posB <= myAngleUnLevel; posB++) { // Level Bucket higher (Let go of items in scooper)
    myServoB.write(posB);
    delay(15);
  }
  delay(1500);
  for (posB = myAngleUnLevel; posB >= myAngleB1; posB--) { // Level Bucket higher (Let go of items in scooper)
    myServoB.write(posB);
    delay(15);
  }

}

//180 Turn of Minibot
void turn180 (int delta) {
  //if the Minibot picking up, move back further
  //if Minibot is dropping off, move back less far.
  if (count % 2 != 0) {
    del = 2200;
  }
  else {
    del = 1800;
  }

  //Run motors back wards to make room between orange cones and bot
  runMotors(-11, -10 + 1);
  delay(del);
  runMotors(0, 0);
  delay(300);

  //if mini bot is picking up lift bucket to make room to turn
  if (count % 2 != 0) {
    liftBucket();
  }

  turnRover(6, 11 );
  delay(2200);
  fixWheels();
}

// Pickup function
void pickUp() {
  runMotors(0, 0);
  delay(100);
  dropBucket();
  runMotors(-11, -10 + 1);
  delay(1600);
  runMotors(0, 0);
  delay(100);
  liftBucket();
  runMotors(0, 0);
  delay(100);

}

//Drop-off function
void dropOff() {
  runMotors(0, 0);
  delay(200);
  runMotors(-11, -10 + 1);
  delay(1000);
  runMotors(0, 0);
  delay(500);
  unlevelBucket();


}
