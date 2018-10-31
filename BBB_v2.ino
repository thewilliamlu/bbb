/***********************[NOTICE]*********************************
Big Basket Boys Automated Shopping Basket (Fall 2018) 
Team: An Chung, Muhammad Hamza, Jason Ngo, William Lu, Matthew Ramirez
Instructor: Seepersad
Course ME 366J (18370) Mechanical Design and Methodology 

Last updated: 10/27/2018

Requires 
- 2 motors (left and right)
- 3 ultrasonic sensors (front (1); left (2); right (3)
- 1 Pixy Camera

MUST ADJUST
-motor speed trim to adjust for mismatched motors
-turn distance
-speed
-Pixy2 object recognition camera parameters
  +signature color set
  +steering differential 
  +overall motor speed (speed up if too far away) (???)

Integrating Pixy2 to Self-Driving Car 
****************************************************************/
#define leftFactor 0 //adds trim speed to left motor 
#define rightFactor 30 //adds trim speed to right motor 

#define speedSet  150 //defines speed of both motors
#define TURN_DIST 30 //defines range between object and car

#define X_CENTER 160L //defines center of frame on x-axis
int RightMotorSpeed = 0; 
int LeftMotorSpeed = 0; 
int motorAcceleration = 0;
long pillow = 1000; //following distance to object
uint32_t lastBlockTime = 0; 

#include <AFMotor.h> //header file allowing access to AFMotor functions

#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <SPI.h>

Pixy2 pixy;

AF_DCMotor leftMotor(3, MOTOR34_64KHZ); //constructor for leftmotor selecting pin 3 and frequency for channel 3 and 4
AF_DCMotor rightMotor(4, MOTOR34_64KHZ); //constructor for rightmotor selecting pin 4 and frequency for channel 3 and 4
 
//global variables
int trig1 = A0; //analog input pin used as digital pin A0
int echo1 = A1; //analog input pin used as digital pin A1
int trig2 = A2; //analog input pin used as digital pin A2
int echo2 = A3; //analog input pin used as digital pin A3
int trig3 = A4; //analog input pin used as digital pin A4
int echo3 = A5; //analog input pin used as digital pin A5

unsigned int S1; //global variable S1 used for distance in front
unsigned int S2; //global variable S2 used for distance to the left
unsigned int S3; //global variable S3 used for distance to the right

void setup() {
  Serial.begin(9600); //set data rate in baud (bits per second) for data transmission 
  pinMode(trig1, OUTPUT); //configure pin A2 as output pin with low impedance to provide large current
  pinMode(echo1, INPUT); //configure pin A3 as input pin with high impedance to read sensor data
  pinMode(trig2, OUTPUT); 
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT); 
  pinMode(echo3, INPUT);
  delay(2000); //delay 2000 ms
  pixy.init(); //initialize the Pixy2 module with board
  pixy.setLamp(1,0); //turn on Pixy2 lamp
}

void loop() {
 range1();
 
 if(S1 <= TURN_DIST) {
  turn();
 } 
 else {
    uint16_t blocks; 
    blocks = pixy.ccc.getBlocks(); //array of blocks detected by Pixy2
    if (blocks) {
      int trackedBlock = TrackBlock(blocks);
      FollowBlock(trackedBlock); //left & right steering to object and speedup & slow down
      moveForward();
      lastBlockTime = millis(); //count time when last block was detected
    }
      else if (millis() - lastBlockTime > 5000) {
        moveStop(); //stop car if nothing there for too long
      }
        else {
          RightMotorSpeed = 0; 
          LeftMotorSpeed = 0; 
          motorAcceleration = 0;
          moveForward(); //forward move function blindly for 5 seconds
        }
 }
}

//return index of block to track
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0; //minumum size
    
  for (int i = 0; i < blockCount; i++)
  {
    //find the largest block of Signature #1
    if (pixy.ccc.blocks[i].m_signature == 1) //we will only use Signature #1 
    {
      long newSize = pixy.ccc.blocks[i].m_height * pixy.ccc.blocks[i].m_width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize; 
      }
    }
  }
  return trackedBlock;
}

//function to follow block
void FollowBlock(int trackedBlock)
{
  int32_t followError = X_CENTER - pixy.ccc.blocks[trackedBlock].m_x;  // How far off-center are we looking now?

  // Size is the area of the object.
  long blockSize = 0;
  blockSize = pixy.ccc.blocks[trackedBlock].m_width * pixy.ccc.blocks[trackedBlock].m_height; 

  //Speed control (implement overall motor speed to "catch up" to object
  if (blockSize > pillow)//closer than threshold
  {
    motorAcceleration = -50;
  }
    else if (blockSize < pillow) //further than threshold
    {
      motorAcceleration = 50;
    }


  //Steering control (implmenent motor speed differential here:
  if (followError < -50)
  {
    //Serial.println("steer left");
    RightMotorSpeed = 30; 
    LeftMotorSpeed = -30; 
  }
    else if (followError > 50)
      {
        //Serial.println("steer right");
        RightMotorSpeed = -30; 
        LeftMotorSpeed = 30; 
      }
      else
      {
        //Serial.println("keep going forward");
        RightMotorSpeed = 0; 
        LeftMotorSpeed = 0; 
      }
}

void turn() { //declare turn function
  moveStop(); //stop the motor (car)
  range1(); //call range function to find S1 distance front
  range2(); //call range function to find S2 distance to the left
  range3(); //call range function to find S2 distance to the right
  if (S2 <= TURN_DIST && S3 <= TURN_DIST) { //if both left and right distances are too close, move backwards
    //ex if in corner, move backwards
    moveBackward(); //both motors run in reverse car goes backwards
    delay(500); //delay 500ms
    if (S2 >= S3) { //if distance to nearest object on leftside is greater than rightside
      turnLeft(); //turn left
    } else { //if distance to nearest object on rightside is greater than leftside
      turnRight(); //turn right
    }
  }
}

void range1(){
  digitalWrite(trig1, LOW); //write the pin to int trig and low to ground r 0V
  delayMicroseconds(2); //delay 2 microseconds
  digitalWrite(trig1, HIGH); //write pin to int trig and high to 5V or 3.33V
  delayMicroseconds(20); //delay 20 microseconds
  digitalWrite(trig1, LOW); ///write the pin to int trig and low to ground or 0V
  int distance = pulseIn(echo1, HIGH); //reads pulse in microseconds from high to low and sets value as distance
  distance = distance / 58; //give distance in cm
  S1 = distance; //S is the distance variable in cm and is variable used as distance convention
  if (S1 < TURN_DIST) { //if the found distance S is less than threshold for "too close" 
    delay(50); //delay 50 ms 
  }
}

void range2(){
  digitalWrite(trig2, LOW); //write the pin to int trig and low to ground r 0V
  delayMicroseconds(2); //delay 2 microseconds
  digitalWrite(trig2, HIGH); //write pin to int trig and high to 5V or 3.33V
  delayMicroseconds(20); //delay 20 microseconds
  digitalWrite(trig2, LOW); ///write the pin to int trig and low to ground or 0V
  int distance = pulseIn(echo2, HIGH); //reads pulse in microseconds from high to low and sets value as distance
  distance = distance / 58; //give distance in cm
  S2 = distance; //S is the distance variable in cm and is variable used as distance convention
  if (S2 < TURN_DIST) { //if the found distance S is less than threshold for "too close" 
    delay(50); //delay 50 ms 
  }
}

void range3(){
  digitalWrite(trig3, LOW); //write the pin to int trig and low to ground r 0V
  delayMicroseconds(2); //delay 2 microseconds
  digitalWrite(trig3, HIGH); //write pin to int trig and high to 5V or 3.33V
  delayMicroseconds(20); //delay 20 microseconds
  digitalWrite(trig3, LOW); ///write the pin to int trig and low to ground or 0V
  int distance = pulseIn(echo3, HIGH); //reads pulse in microseconds from high to low and sets value as distance
  distance = distance / 58; //give distance in cm
  S3 = distance; //S is the distance variable in cm and is variable used as distance convention
  if (S3 < TURN_DIST) { //if the found distance S is less than threshold for "too close" 
    delay(50); //delay 50 ms 
  }
}

void moveForward() { //declare forward movement function
  leftMotor.run(FORWARD); //run leftmotor forward
  rightMotor.run(FORWARD); //run rightmotor forward
  leftMotor.setSpeed(speedSet + leftFactor + LeftMotorSpeed + motorAcceleration); //set leftmotor speed 
  rightMotor.setSpeed(speedSet + rightFactor + RightMotorSpeed + motorAcceleration); //set right motor speed
}

void turnLeft() { //declare left turn function
  leftMotor.run(BACKWARD); //run leftmotor backwards
  rightMotor.run(FORWARD); //run rightmotor forwards
  leftMotor.setSpeed(speedSet + leftFactor); //set leftmotor speed as speedSet + correction factor for left motor 
  rightMotor.setSpeed(speedSet + rightFactor);//set right motor speed as speedSet + correction factor for right motor
  delay(700); //delay 700ms to turn for set time until stop
  moveStop(); //call stop function to stop the car
}
void turnRight() { //declare right turn function
  leftMotor.run(FORWARD); //run leftmotor foward
  rightMotor.run(BACKWARD); //run rightmotor backwards
  leftMotor.setSpeed(speedSet + leftFactor);//set leftmotor speed as speedSet + correction factor for left motor 
  rightMotor.setSpeed(speedSet + rightFactor);//set right motor speed as speedSet + correction factor for right motor
  delay(700); //delay 700ms to turn for set time until stop
  moveStop(); //call stop function to stop car
}
void moveBackward() { //declare backward move function
  leftMotor.run(BACKWARD); //run leftmotor backwards
  rightMotor.run(BACKWARD); //run rightmotor backwards
  leftMotor.setSpeed(speedSet + leftFactor);//set leftmotor speed as speedSet + correction factor for left motor 
  rightMotor.setSpeed(speedSet + rightFactor);//set right motor speed as speedSet + correction factor for right motor
}

void moveStop() { //declare stop function to stop the car
  leftMotor.run(RELEASE); rightMotor.run(RELEASE); //release removes power from the motor to stop both left and right
}
  

