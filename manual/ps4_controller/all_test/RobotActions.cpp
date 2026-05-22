#include "RobotActions.h"

// --- State Variables for Turning ---
bool isTurning = false;
long targetTicksLeft = 0;
long targetTicksRight = 0;
int turnDirection = 0;

// --- State Variables for Moving Straight ---
bool isMovingStraight = false;
long targetMoveTicksLeft = 0;
long targetMoveTicksRight = 0;
int moveDirection = 0;

// --- State Variables for Elevator ---
bool isElevatorMoving = false;
long targetElevatorTicks = 0;
int elevatorDirection = 0;

// ==========================================
// LOCOMOTION ACTIONS
// ==========================================
void initiateTurn(float degrees) {
  float arcLength = (abs(degrees) / 360.0) * PI * TRACK_WIDTH_MM;
  float wheelCircumference = PI * WHEEL_DIA_MM;
  long requiredTicks = (arcLength / wheelCircumference) * CPR;

  isTurning = true;

  if (degrees > 0) {
    // RIGHT TURN (Left wheel forward, Right wheel backward)
    turnDirection = 1;
    targetTicksLeft = ticksLeft + requiredTicks;
    targetTicksRight = ticksRight - requiredTicks;
    motorLeft.setPower(150);
    motorRight.setPower(-150);
  } else {
    // LEFT TURN (Left wheel backward, Right wheel forward)
    turnDirection = -1;
    targetTicksLeft = ticksLeft - requiredTicks;
    targetTicksRight = ticksRight + requiredTicks;
    motorLeft.setPower(-150);
    motorRight.setPower(150);
  }
}

bool checkTurnComplete() {
  if (!isTurning) return true;

  bool leftDone = false;
  bool rightDone = false;

  if (turnDirection == 1) { // Checking RIGHT turn
    if (ticksLeft >= targetTicksLeft) { motorLeft.stop(); leftDone = true; }
    if (ticksRight <= targetTicksRight) { motorRight.stop(); rightDone = true; }
  } else {                  // Checking LEFT turn
    if (ticksLeft <= targetTicksLeft) { motorLeft.stop(); leftDone = true; }
    if (ticksRight >= targetTicksRight) { motorRight.stop(); rightDone = true; }
  }

  if (leftDone && rightDone) {
    isTurning = false;
    return true;
  }
  return false;
}

void initiateMove(float centimeters) {
  // Convert cm to mm for calculation
  float distanceMM = abs(centimeters) * 10.0; 
  float wheelCircumference = PI * WHEEL_DIA_MM;
  
  // Calculate how many ticks are needed to travel that distance
  long requiredTicks = (distanceMM / wheelCircumference) * CPR;

  isMovingStraight = true;

  if (centimeters > 0) {
    // Moving FORWARD
    moveDirection = 1;
    targetMoveTicksLeft = ticksLeft + requiredTicks;
    targetMoveTicksRight = ticksRight + requiredTicks;
    motorLeft.setPower(150);
    motorRight.setPower(150);
  } else {
    // Moving BACKWARD
    moveDirection = -1;
    targetMoveTicksLeft = ticksLeft - requiredTicks;
    targetMoveTicksRight = ticksRight - requiredTicks;
    motorLeft.setPower(-150);
    motorRight.setPower(-150);
  }
}

bool checkMoveComplete() {
  if (!isMovingStraight) return true;

  bool leftDone = false;
  bool rightDone = false;

  if (moveDirection == 1) { // Checking FORWARD movement
    if (ticksLeft >= targetMoveTicksLeft) { motorLeft.stop(); leftDone = true; }
    if (ticksRight >= targetMoveTicksRight) { motorRight.stop(); rightDone = true; }
  } else {                  // Checking BACKWARD movement
    if (ticksLeft <= targetMoveTicksLeft) { motorLeft.stop(); leftDone = true; }
    if (ticksRight <= targetMoveTicksRight) { motorRight.stop(); rightDone = true; }
  }

  // If both wheels have hit their targets, the move is complete
  if (leftDone && rightDone) {
    isMovingStraight = false;
    return true;
  }
  
  return false;
}

// ==========================================
// INTAKE ACTIONS
// ==========================================
void setGripperState(bool clamp) {
  long stepsToClose = 800; 
  int stepSpeed = 500; 
  if (clamp) stepperMotor.move(stepsToClose, stepSpeed);
  else stepperMotor.move(-stepsToClose, stepSpeed);
}

// Manual infinite control
void moveElevator(int direction) {
  float liftSpeed = 400.0; 
  if (direction > 0) heavyMotor.setTargetSpeed(liftSpeed);   
  else if (direction < 0) heavyMotor.setTargetSpeed(-liftSpeed);  
  else heavyMotor.setTargetSpeed(0.0);         
}

// Precise Margin Control
void moveElevatorByMargin(float centimeters) {
  long ticksToMove = centimeters * ELEVATOR_TICKS_PER_CM;
  targetElevatorTicks = ticksHeavy + ticksToMove;
  isElevatorMoving = true;
  
  if (centimeters > 0) {
    elevatorDirection = 1;
    heavyMotor.setTargetSpeed(400.0); // Move up at 400 ticks/sec
  } else {
    elevatorDirection = -1;
    heavyMotor.setTargetSpeed(-400.0); // Move down at 400 ticks/sec
  }
}

// Check if the margin is reached
bool checkElevatorComplete() {
  if (!isElevatorMoving) return true;

  bool targetReached = false;

  // If moving UP, check if current ticks are greater than target
  if (elevatorDirection == 1 && ticksHeavy >= targetElevatorTicks) {
    targetReached = true;
  }
  // If moving DOWN, check if current ticks are less than target
  else if (elevatorDirection == -1 && ticksHeavy <= targetElevatorTicks) {
    targetReached = true;
  }

  if (targetReached) {
    heavyMotor.setTargetSpeed(0.0); // Stop the elevator
    isElevatorMoving = false;
    return true;
  }
  
  return false;
}