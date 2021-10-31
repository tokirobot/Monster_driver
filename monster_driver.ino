/*  MonsterMoto Shield Example Sketch
  date: 5/24/11
  code by: Jim Lindblom
  hardware by: Nate Bernstein
  SparkFun Electronics
 This is really simple example code to get you some basic
 functionality with the MonsterMoto Shield. The MonsterMote uses
 two VNH2SP30 high-current full-bridge motor drivers.
 
 Use the motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) 
 function to get motors going in either CW, CCW, BRAKEVCC, or 
 BRAKEGND. Use motorOff(int motor) to turn a specific motor off.
 
 The motor variable in each function should be either a 0 or a 1.
 pwm in the motorGo function should be a value between 0 and 255.
 
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given.
 */

#include <ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "stdlib.h"
#include <string>






#define BRAKEVCC 0
#define UP  1
#define DOWN  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define LEFTMOTOR 0
#define RIGHTMOTOR 1

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

int statpin = 13;
int moveflag=0;
int flipperspeed=0;
int flippercommand=0;

enum mode {
    Asynchronous=1,
    Synchronous,
    DynamixelArm
};

enum command{
    STOP=0,

    LFU=10,
    LFD=20,
    
    LBU=30,
    LBD=40,
    
    RFU=50,
    RFD=60,
    
    RBU=70,
    RBD=80,

    LRFU=90,
    LRFD=100,
    
    LRBU=110,
    LRBD=120,

    FRONT=130,
    BACK=140,
    RIGHT=150,
    LEFT=160,

};

std_msgs::Int32MultiArray mtcmd;
std_msgs::Float32MultiArray IMU_data;
//1:crawler 2:flipper 3:flippper speed 4:crawler speed
void back_flip_callback(const std_msgs::Int32MultiArray &flipcmd){
    mtcmd.data[0] = flipcmd.data[0];
    mtcmd.data[1] = flipcmd.data[1];
    mtcmd.data[2] = flipcmd.data[2];
    mtcmd.data[3] = flipcmd.data[3];
    moveflag = flipcmd.data[4];

    flipperspeed=mtcmd.data[2];
    flippercommand=mtcmd.data[1];
  
}
ros::NodeHandle nh;



ros::Subscriber<std_msgs::Int32MultiArray> sub("opencr_fli", &back_flip_callback);
ros::Publisher IMU_Publisher("imu_pub",&IMU_data);



void setup()
{
  Serial.begin(9600);
   nh.initNode();
   nh.advertise(IMU_Publisher);
   nh.subscribe(sub);
   

  
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  // motorGo(0, CW, 1023);
  // motorGo(1, CCW, 1023);
}

void loop()
{
  //motorGo(0, CW, 1023);
  //motorGo(1, CCW, 1023);
 // delay(500);

  //motorGo(0, CCW, 1023);
 // motorGo(1, CW, 1023);
  //delay(500);
  Serial.println(flipperspeed);
  Serial.println(flippercommand);
  flipper_drive(flipperspeed,flippercommand);
  
  if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD))
    digitalWrite(statpin, HIGH);
     nh.spinOnce();
}

void motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void driveflipper(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
/*STOP=0,

    LFU=10,
    LFD=20,
    
    LBU=30,
    LBD=40,
    
    RFU=50,
    RFD=60,
    
    RBU=70,
    RBD=80,

    LRFU=90,
    LRFD=100,
    
    LRBU=110,
    LRBD=120,

    FRONT=130,
    BACK=140,
    RIGHT=150,
    LEFT=160,*/
/*#define BRAKEVCC 0
#define UP  1
#define DOWN  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define LEFTMOTOR 0
#define RIGHTMOTOR 1*/

  // motorGo(0, CW, 1023);
  // motorGo(1, CCW, 1023);
void flipper_drive(int velosity, int command){
    switch (command)
  {
      case LBU:
      driveflipper(LEFTMOTOR,UP,velosity);
      break;

      case LBD:
      driveflipper(LEFTMOTOR,DOWN,velosity);
      break;
      
      case RBU:
      driveflipper(RIGHTMOTOR,UP,velosity);
      break;
      
      case RBD:
      driveflipper(RIGHTMOTOR,DOWN,velosity);
      break;
      
      case LRBU:
      driveflipper(LEFTMOTOR,UP,velosity);
      driveflipper(RIGHTMOTOR,UP,velosity);
      break;
      
      case LRBD:
      driveflipper(LEFTMOTOR,DOWN,velosity);
      driveflipper(RIGHTMOTOR,DOWN,velosity);
      break; 
         
      case STOP:
      driveflipper(LEFTMOTOR,DOWN,0);
      driveflipper(RIGHTMOTOR,DOWN,0);
        break;
  

  }
  
}
