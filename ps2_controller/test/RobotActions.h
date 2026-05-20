#ifndef ROBOT_ACTIONS_H
#define ROBOT_ACTIONS_H

#include <Arduino.h>
#include "MotorDrivers.h"

// --- Hardware Globals ---
extern L298N_Motor motorLeft;
extern L298N_Motor motorRight;
extern MOSFET_Motor heavyMotor;
extern A4988_Stepper stepperMotor;

extern volatile long ticksLeft;
extern volatile long ticksRight;
extern volatile long ticksHeavy;

extern bool isMovingStraight;
extern bool isTurning;
extern bool isElevatorMoving;

// --- Kinematics Constants ---
const float TRACK_WIDTH_MM = 200.0; 
const float WHEEL_DIA_MM = 65.0;    
const float CPR = 360.0; 

// YOU MUST CALIBRATE THIS: How many ticks equal 1 cm of lift?
const float ELEVATOR_TICKS_PER_CM = 150.0; 

// --- Function Declarations ---
void initiateTurn(float degrees);
bool checkTurnComplete();
void initiateMove(float centimeters);
bool checkMoveComplete();

void setGripperState(bool clamp);
void moveElevator(int direction); // Infinite move (from earlier)

void moveElevatorByMargin(float centimeters);
bool checkElevatorComplete();

#endif