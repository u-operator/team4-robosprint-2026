// ==========================================
// ARDUINO MEGA PIN CONFIGURATION
// Wiring is shared with the "intake and locomotion" firmware.
// Both sketches target the same Arduino Mega and the same harness.
// ==========================================

#ifndef PIN_CONF
#define PIN_CONF

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

// Limit Switches
#define LIMIT_TOP1 51
#define LIMIT_TOP2 53

// Relay
#define RLYL 49    // Left
#define RLYR 37   // Right

#endif