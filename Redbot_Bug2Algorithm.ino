/*
ECE 452 Robotics Algo. and Control
Project 2
Team 7
TAK'S JOSEPH REFUGIO
ZACH HELLREIGEL
MATTHEW GOSTKOWSKI
5/13/2018
*/
#include <RedBot.h>   // This line "includes" the RedBot library in the sketch.
                      // Provides special objects, methods, and functions for the RedBot.

// FUNCTION PROTOTYPES
void checkDistance();
void followLine();
void goStraight(int);
void goLeft();
void turnAngle(int);

RedBotMotors motors;                                // initialize the motor control object.
RedBotAccel accelerometer;                            // initialize accelerometer object
RedBotSensor leftSensor = RedBotSensor(A3);           // initialize a left sensor object on A3
RedBotSensor centerSensor = RedBotSensor(A6);           // initialize a center sensor object on A6
RedBotSensor rightSensor = RedBotSensor(A7);            // initialize a right sensor object on A7
RedBotEncoder encoder = RedBotEncoder(A2, 10);        // initializes encoder on pins A2 and 10

#define LINETHRESHOLD 800   // LINETHRESHOLD is the level to detect if the sensor is on the line or not. 
                            // If the sensor value is greater than this the sensor is above a DARK line.
#define SPEED 110           // sets the nominal speed. Set to any number from 0 - 255.
#define countsPerRev 192    // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

// WAIT: the robot motors are stopped and GO mode enabled when accelerometer senses a change 
// GO: robot follows line and enables STOP mode when distance of 50" is detected
// STOP: Robot motors stop
typedef enum MODE {WAIT, GO, STOP};
MODE robotMode = WAIT;                  // initial mode is WAIT 

int leftSpeed;                          // variable used to store the leftMotor speed
int rightSpeed;                         // variable used to store the rightMotor speed

float wheelDiam = 6.5;                // diam of wheel in cm
float wheelCirc = PI*wheelDiam;         // Redbot wheel circumference = pi*D
float distance = 50;                  // target disance in cm
float currentDistance;                  // variable stores current distance in GO mode

long lCount = 0;         // left wheel encoder tick count
long rCount = 0;         // right wheel encoder tick count
long maxCount;           // maxCount stores the maximum encoder tick count 

/**********************************************MAIN SETUP*************************************************************/
void setup(void) {
  //Starts Serial Monitor
  Serial.begin(9600);
  delay(2000);
}
  
void loop(void) {
  if (robotMode == WAIT) {
    encoder.clearEnc(BOTH);                           // clear the encoder count of both wheels
    accelerometer.read();                             // read accelerometer positions
    if (accelerometer.x > 5000) { robotMode = GO; }   // when x position is greater than 5000, enable GO mode
  }

  if (robotMode == GO) { 
  	turnAngle(90);
  	delay(500);
  	goStraight(4);
  	delay(500);
  	turnAngle(90);
  	while(true) { followLine(); Serial.println(currentDistance);}
  	turnAngle(150);
  	goStraight(30);
    robotMode = STOP;
  }                

  if (robotMode == STOP) { motors.stop(); }           // robot stops when robot follows line for 50"
}
/********************************************************************************************************************/
// Drives the Robot Forward
void goStraight(int travelDist) {
  encoder.clearEnc(BOTH);
  int rightTicksNow = 0;
  int leftTicksNow = 0;
  int maxTicks = 0;
  float nowDistance = 0;

  while (nowDistance < travelDist)
  {

    rightTicksNow = encoder.getTicks(RIGHT);
    leftTicksNow = encoder.getTicks(LEFT); 
    
    if (rightTicksNow > leftTicksNow) { maxTicks = rightTicksNow; }
    else { maxTicks = leftTicksNow; }

    nowDistance = ((float)maxTicks/countsPerRev)*wheelCirc;

    motors.drive(SPEED);
    // motors.leftMotor(-SPEED);
    // motors.rightMotor(SPEED);
  }
    motors.leftMotor(0);
    motors.rightMotor(0);
}

// checkDistance:
// Read wheel encoder tick count and set maxCount to the maximum tick count
// Update the current distance traveled in inches
// If 50 inches is met then stop.
void checkDistance(void) {
  // read ecoders on both wheels
  lCount = encoder.getTicks(LEFT);
  rCount = encoder.getTicks(RIGHT);

  // set max ecoder tick count
  if (lCount <= rCount){maxCount = rCount;}
  else {maxCount = lCount;}

  // calculate distance in inches
  currentDistance = ((float)maxCount/countsPerRev)*wheelCirc;
}

// Tells the robot to follow the line
void followLine(void) {
  int sensorConfig = 0;
  typedef enum State {ON,OFF};
  State delayTrigger = OFF;

  typedef enum Robotmotion {FORWARD, STOP, LEFT, RIGHT};
  Robotmotion motion = STOP;

  if      (((leftSensor.read() < LINETHRESHOLD) && (centerSensor.read() < LINETHRESHOLD)) && (rightSensor.read() < LINETHRESHOLD)) { motion = FORWARD; }
  else if (((leftSensor.read() < LINETHRESHOLD) && (centerSensor.read() < LINETHRESHOLD)) && (rightSensor.read() > LINETHRESHOLD)) { motion = RIGHT; }
  else if (((leftSensor.read() < LINETHRESHOLD) && (centerSensor.read() > LINETHRESHOLD)) && (rightSensor.read() < LINETHRESHOLD)) { motion = FORWARD; }
  else if (((leftSensor.read() < LINETHRESHOLD) && (centerSensor.read() > LINETHRESHOLD)) && (rightSensor.read() > LINETHRESHOLD)) { motion = RIGHT; }

  else if (((leftSensor.read() > LINETHRESHOLD) && (centerSensor.read() < LINETHRESHOLD)) && (rightSensor.read() < LINETHRESHOLD)) { motion = LEFT; }
  else if (((leftSensor.read() > LINETHRESHOLD) && (centerSensor.read() < LINETHRESHOLD)) && (rightSensor.read() > LINETHRESHOLD)) { motion = STOP; }
  else if (((leftSensor.read() > LINETHRESHOLD) && (centerSensor.read() > LINETHRESHOLD)) && (rightSensor.read() < LINETHRESHOLD)) { motion = LEFT; }
  else if (((leftSensor.read() > LINETHRESHOLD) && (centerSensor.read() > LINETHRESHOLD)) && (rightSensor.read() > LINETHRESHOLD)) { motion = LEFT; }

  switch (motion)
  {
    case FORWARD:
      leftSpeed = -SPEED;
      rightSpeed = SPEED;
      break;
    case LEFT:
      leftSpeed = 0;
      rightSpeed = SPEED;
      break;
    case RIGHT:
      leftSpeed = -SPEED;
      rightSpeed = 0;
      break;
    case STOP:
      leftSpeed = 0;
      rightSpeed = 0;
      delayTrigger = ON;
      break;
    default:
      leftSpeed = 0;
      rightSpeed = 0;
      break;
  }  
  
  motors.leftMotor(leftSpeed);
  motors.rightMotor(rightSpeed); 

  // update the distance traveled
  checkDistance();
  delay(0);
}

// Parks the robot when it reaches the goal
void goLeft(void) {
  int now_rticks = encoder.getTicks(RIGHT);
  int rticks = encoder.getTicks(RIGHT);
  int targetTicks = 250 + now_rticks;                      // 220: number of ticks it takes to turn left

  while (rticks < targetTicks)
  {
    motors.leftMotor(0);
    motors.rightMotor(SPEED);
    rticks = encoder.getTicks(RIGHT);   // update the right ticks
  }

  // Stop the robot because it's done turning
  motors.leftMotor(0);
  motors.rightMotor(0);
  delay(1000);
}

// Takes in an angle in radians and makes the robot pivot about its 
// center. Positive angle pivots the robot clockwise; negative angle 
// pivots the robot counter clockwise
void turnAngle(int angle) {
	encoder.clearEnc(BOTH);	

	int angleInTicks = (angle)*(110/90);
	int rticks = encoder.getTicks(RIGHT);
	int now_rticks = rticks;
	int targetTicks = angleInTicks + now_rticks;

	while(rticks < targetTicks) 
	{
		motors.leftMotor(SPEED);
	    motors.rightMotor(SPEED);
	    rticks = encoder.getTicks(RIGHT);   // update the right ticks
	}

	 // Stop the robot because it's done turning
	  motors.leftMotor(0);
	  motors.rightMotor(0);
}