/***********************[NOTICE]*********************************
Big Basket Boys Automated Shopping Basket (Fall 2018) 
Team: An Chung, Muhammad Hamza, Jason Ngo, William Lu, Matthew Ramirez
Instructor: Seepersad
Course ME 366J (18370) Mechanical Design and Methodology 

Last updated: 10/27/2018

Purpose: To test Pixy2 module and print value of motor speeds as object with Signature #1
  is tracked. 

  -controls motor differential speed as object is offset from the center of the camera view 
    x-axis offest only
  -controls motor speed to speed up if further away and slow down when too close

Show fine resolution steering
****************************************************************/

#define X_CENTER 160L //defines center of frame on x-axis
int motorSpeed = 0; 
int motorAcceleration = 0;

#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <SPI.h>

Pixy2 pixy;

void setup() {
  Serial.begin(9600); 
  pixy.init(); //initialize the Pixy2 module with board
  pixy.setLamp(1,0); //turn on Pixy2 lamp
}

uint32_t lastBlockTime = 0; 
  
void loop() {
  uint16_t blocks; 
  blocks = pixy.ccc.getBlocks(); //array of blocks detected by Pixy2

  //track and follow a block
  if (blocks){
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis(); //count time when last block was detected
    Serial.print(motorSpeed); //represent steering (+3:Right; -3:Left; 0:Straight ahead)
    Serial.print(",");
    Serial.println(motorAcceleration); //Represent distance to object (speed up if far/slow down if close)
  }
  else if(millis() - lastBlockTime > 100) //stop car if no blocks
  {
    //Serial.println("Motors Stopped");
    motorSpeed = 0;
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

  // Size of object pillow at about 1ft away from camera
  long pillow = 10000;
  // Size is the area of the object.
  long blockSize = 0;
  blockSize = pixy.ccc.blocks[trackedBlock].m_width * pixy.ccc.blocks[trackedBlock].m_height; 

  motorAcceleration = blockSize / 10000; //show how "close" the block is

  motorSpeed = followError / 20; //show where the block is with 0=center; negative=right; positive=left
  
}


