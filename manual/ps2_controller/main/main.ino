#include <PS2X_lib.h>
#include "MotorDrivers.h"
#include "RobotActions.h"

// ==========================================
// ARDUINO MEGA PIN CONFIGURATION
// Wiring is shared with the "intake and locomotion" firmware.
// Both sketches target the same Arduino Mega and the same harness.
// ==========================================
// --- L298N (Drive Motors) ---
// Note: ENA/ENB must be on PWM-capable pins (Mega has PWM on 2-13, 44-46)
#define L298N_ENA 5
#define L298N_IN1 30
#define L298N_IN2 32

#define L298N_ENB 6
#define L298N_IN3 34
#define L298N_IN4 36

// --- MOSFET (Heavy Elevator Motor) ---
#define MOSFET_EN   38
#define MOSFET_PWM1 7  // Must be PWM
#define MOSFET_PWM2 8  // Must be PWM

// --- A4988 (Stepper Motor) ---
#define A4988_STEP A0
#define A4988_DIR  A1
#define A4988_EN   A2

// --- ENCODERS (CRITICAL: 'A' Pins MUST be on Interrupts) ---
// Mega Interrupt pins are: 2, 3, 18, 19, 20, 21
#define ENC_L_A 18 // Interrupt 5
#define ENC_L_B 22

#define ENC_R_A 19 // Interrupt 4
#define ENC_R_B 24

#define ENC_H_A 20 // Interrupt 3
#define ENC_H_B 26

// --- PS2 RECEIVER PINS (this firmware only; reserved in the other) ---
#define PS2_DAT 40
#define PS2_CMD 42
#define PS2_ATT 44
#define PS2_CLK 46

// --- Instantiations ---
L298N_Motor  motorLeft(L298N_ENA, L298N_IN1, L298N_IN2);
L298N_Motor  motorRight(L298N_ENB, L298N_IN3, L298N_IN4);
MOSFET_Motor heavyMotor(MOSFET_EN, MOSFET_PWM1, MOSFET_PWM2);
A4988_Stepper stepperMotor(A4988_STEP, A4988_DIR, A4988_EN);

// --- PS2 Controller Object ---
PS2X ps2x;
int error = 0;

// --- Global Volatile Variables ---
volatile long ticksLeft = 0;
volatile long ticksRight = 0;
volatile long ticksHeavy = 0;

// --- ISRs ---
void isr_left() { if (digitalRead(ENC_L_B) == HIGH) ticksLeft++; else ticksLeft--; }
void isr_right() { if (digitalRead(ENC_R_B) == HIGH) ticksRight++; else ticksRight--; }
void isr_heavy() { if (digitalRead(ENC_H_B) == HIGH) ticksHeavy++; else ticksHeavy--; }

void setup() {
  Serial.begin(115200); 
  
  motorLeft.begin(); 
  motorRight.begin(); 
  heavyMotor.begin(); 
  stepperMotor.begin();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_H_A, INPUT_PULLUP); pinMode(ENC_H_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_H_A), isr_heavy, RISING);

  heavyMotor.setPIDConstants(1.5, 0.2, 0.05); 

  // Initialize PS2 Controller
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if (error == 0) Serial.println("Found Controller, configured successfully");
  else Serial.println("No controller found, check wiring!");
}

void loop() {
  // 1. HARDWARE BACKGROUND TASKS
  heavyMotor.computePID(ticksHeavy);
  stepperMotor.update();

  // 2. READ PS2 CONTROLLER
  if (error == 1) return; // Skip if no controller is plugged in
  
  ps2x.read_gamepad(); 

  // ==========================================
  // A. LOCOMOTION (Arcade Drive)
  // ==========================================
  int stickY = ps2x.Analog(PSS_LY); // 0-255
  int stickX = ps2x.Analog(PSS_LX); // 0-255
  
  int drive = map(stickY, 0, 255, 255, -255); 
  int turn  = map(stickX, 0, 255, 255, -255); 

  if (abs(drive) < 20) drive = 0;
  if (abs(turn) < 20) turn = 0;

  motorLeft.setPower(drive + turn);
  motorRight.setPower(drive - turn);

  // ==========================================
  // B. ELEVATOR CONTROL (D-PAD)
  // ==========================================
  if (ps2x.Button(PSB_PAD_UP)) moveElevator(1); 
  else if (ps2x.Button(PSB_PAD_DOWN)) moveElevator(-1); 
  else moveElevator(0); 

  // ==========================================
  // C. GRIPPER CONTROL (Shoulders)
  // ==========================================
  if (ps2x.ButtonPressed(PSB_R1)) setGripperState(true);  // Clamp
  if (ps2x.ButtonPressed(PSB_L1)) setGripperState(false); // Release

  delay(20); 
}