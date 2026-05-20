#include <PS2X_lib.h>
#include "MotorDrivers.h"
#include "RobotActions.h"
#include "protocol.h"

// ==========================================
// CONSTANTS
// Drive (forward/backward) & Turn (left/right)
// Values below this will be clamped to zero
#define DRIVE_THRES 20
#define TURN_THRES 20

// ==========================================
// ARDUINO MEGA PIN CONFIGURATION
// Wiring is shared with the "intake and locomotion" firmware.
// Both sketches target the same Arduino Mega and the same harness.
// ==========================================
// --- L298N (Drive Motors) ---
// Note: ENA/ENB must be on PWM-capable pins (Mega has PWM on 2-13, 44-46)
#define L298N_ENA 6
#define L298N_IN1 29
#define L298N_IN2 27

#define L298N_ENB 5
#define L298N_IN3 25
#define L298N_IN4 23

// --- MOSFET (Heavy Elevator Motor) ---
#define MOSFET_EN   39
#define MOSFET_PWM1 8  // Must be PWM
#define MOSFET_PWM2 7  // Must be PWM

// --- A4988 (Stepper Motor) ---
#define A4988_STEP A1
#define A4988_DIR  A0
#define A4988_EN   A2

// --- ENCODERS (CRITICAL: 'A' Pins MUST be on Interrupts) ---
// Mega Interrupt pins are: 2, 3, 18, 19, 20, 21
//
#define ENC_L_A 19 // Interrupt 5
#define ENC_L_B 33

#define ENC_R_A 18 // Interrupt 4
#define ENC_R_B 31

#define ENC_H_A 20 // Interrupt 3
#define ENC_H_B 35

// --- PS2 RECEIVER PINS
#define PS2_DAT 41
#define PS2_CMD 43
#define PS2_ATT 45
#define PS2_CLK 47

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
  else
  {
    Serial.println("No controller found, check wiring!");
  }
}

void loop()
{
    // 1. HARDWARE BACKGROUND TASKS
  //heavyMotor.computePID(ticksHeavy);
  //stepperMotor.update();

  // 2. READ SERIAL PACKETS

  if (readPacket(Serial, pkt)) {

    switch (pkt.cmd) {

      // ==========================================
      // A. LOCOMOTION (Arcade Drive)
      // ==========================================
      case CMD_MOTORS:
      {
        int leftSpeed  = pkt.b2;
        int rightSpeed = pkt.b4;

        if (pkt.b1 == MT_RVS)
          leftSpeed = -leftSpeed;

        if (pkt.b3 == MT_RVS)
          rightSpeed = -rightSpeed;

        motorLeft.setPower(leftSpeed);
        motorRight.setPower(rightSpeed);

        break;
      }

      case CMD_STOP:
      {
        motorLeft.setPower(0);
        motorRight.setPower(0);

        break;
      }

//       // ==========================================
//       // B. ELEVATOR CONTROL (D-PAD)
//       // ==========================================
//       case CMD_ARM:
//       {
//         if (pkt.b1 == A_UP) {
//           Serial.println("ARM UP");
//           // moveElevator(1);
//         }
//         else {
//           Serial.println("ARM DOWN");
//           // moveElevator(-1);
//         }
//
//         break;
//       }

//       // ==========================================
//       // C. GRIPPER CONTROL (Shoulders)
//       // ==========================================
//       case CMD_GRIP:
//       {
//         if (pkt.b1 == G_CLOSE) {
//           Serial.println("GRIP CLOSE");
//           // setGripperState(true);
//         }
//         else {
//           Serial.println("GRIP OPEN");
//           // setGripperState(false);
//         }

//         break;
//       }
//
//       case CMD_SERVO:
//       {
//         Serial.print("SERVO ");
//         Serial.print(pkt.b1);
//         Serial.print(" ANGLE ");
//         Serial.println(pkt.b2);
//
//         break;
//       }

      default:
      {
        Serial.println("ERR");
        break;
      }
    }
  }
}