// ArduCAM Smart_Robot_Car demo (C)2017
//Before using this demo, you shold install AFMotor library
//which are in the InstallLibrary folder.
// This demo support smart mode .
//video link: https://youtu.be/0FB7J-Qzcag

/***********************[NOTICE]*********************************
  We can't guarantee that the motor load
  is exactly the same, so it increases the compensation
  factor. You should adjust them to suit for your motor
****************************************************************/
#define leftFactor 0 //adds trim speed to left motor 
#define rightFactor 30 //adds trim speed to right motor 

#define speedSet  150 //defines speed of both motors
#define TURN_DIST 30 //defines range between object and car

//include <Servo.h> //header file allowing access to functions for Servo
include <AFMotor.h> //header file allowing access to AFMotor functions

//Servo neckControllerServoMotor; //declare front motor for ultrasonic sensor 
AF_DCMotor leftMotor(3, MOTOR34_64KHZ); //constructor for leftmotor selecting pin 3 and frequency for channel 3 and 4
AF_DCMotor rightMotor(4, MOTOR34_64KHZ); //constructor for rightmotor selecting pin4 and frequency for channel 3 and 4
//global variables
int trig = A2; //analog input pin used as digital pin A2
int echo = A3; //analog input pin used as digital pin A3
unsigned int S; //global variable S used for distance 
unsigned int Sleft; //global variable Sleft used for distance to the left found by range function directed to left of car
unsigned int Sright; //global variabl Sright used for distance to right found by range function difrect to right of car

void setup() { //declare setup
  Serial.begin(9600); //set data rate in baud (bits per second) for data transmission 
  pinMode(trig, OUTPUT); //configure pin A2 as output pin with low impedance to provide large current
  pinMode(echo, INPUT); //configure pin A3 as input pin with high impedance to read sensor data
  //neckControllerServoMotor.attach(10); //attach front motor to pin 10
  //neckControllerServoMotor.write(90); //set center position 
  delay(2000); //delay 2000 ms
}
void loop() { //declare loop function to keep car moving around environment
  //neckControllerServoMotor.write(90); //set center position 
  range(); //call range function
  if (S <= TURN_DIST ) { //if car is too close to object
    turn(); //car will turn
  } else if (S > TURN_DIST) { //else if car is away from object
    moveForward(); //car will keep moving forward
  }
}
void turn() { //declare turn function
  moveStop(); //stop the motor (car)
  neckControllerServoMotor.write(150); //set left position
  delay(500); //delay 500ms after ultrasonic sensor turns left
  range(); //call range function to find S
  Sleft = S; //set Sleft variable to distance S
  neckControllerServoMotor.write(90); //set center position 
  delay(500); //delay 500ms
  neckControllerServoMotor.write(30); //set right position
  delay(500); //delay 500ms
  range(); //call range function to find S
  Sright = S; //set Sright variable as distance S
  neckControllerServoMotor.write(90); //set center position 
  delay(500); //delay 500ms
  if (Sleft <= TURN_DIST && Sright <= TURN_DIST) { //if both left and right distances are too close, move backwards
    //ex if in corner, move backwards
    moveBackward(); //both motors run in reverse car goes backwards
    delay(500); //delay 500ms
    int x = random(1); //set int x as a random int 0 or 1
    if (x = 0) { //if x is 0
      turnRight(); //car turns right
    }
    else { //if x is 1
      turnLeft(); //car turns left
    }
  } else {
    if (Sleft >= Sright) { //if distance to nearest object on leftside is greater than rightside
      turnLeft(); //turn left
    } else { //if distance to nearest object on rightside is greater than leftside
      turnRight(); //turn right
    }
  }
}
void range() { //declare range function to find distance S between sensor and object
  digitalWrite(trig, LOW); //write the pin to int trig and low to ground r 0V
  delayMicroseconds(2); //delay 2 microseconds
  digitalWrite(trig, HIGH); //write pin to int trig and high to 5V or 3.33V
  delayMicroseconds(20); //delay 20 microseconds
  digitalWrite(trig, LOW); ///write the pin to int trig and low to ground or 0V
  int distance = pulseIn(echo, HIGH); //reads pulse in microseconds from high to low and sets value as distance
  distance = distance / 58; //give distance in cm
  S = distance; //S is the distance variable in cm and is variable used as distance convention
  if (S < TURN_DIST) { //if the found distance S is less than threshold for "too close" 
    delay(50); //delay 50 ms 
  }
}
void moveForward() { //declare forward movement function
  leftMotor.run(FORWARD); //run leftmotor forward
  rightMotor.run(FORWARD); //run rightmotor forward
  leftMotor.setSpeed(speedSet + leftFactor); //set leftmotor speed as speedSet + correction factor for left motor 
  rightMotor.setSpeed(speedSet + rightFactor); //set right motor speed as speedSet + correction factor for right motor
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

