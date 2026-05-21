#include "pin_config.h"
#include "protocol.h"
#include "MotorDrivers.h"
#include "RobotActions.h"

volatile long ticksHeavy = 0;
Packet pkt;
long armTarget = 0;
int stepSize = 50;

// --- Instantiations ---
L298N_Motor motorLeft(L298N_ENA, L298N_IN1, L298N_IN2);
L298N_Motor motorRight(L298N_ENB, L298N_IN3, L298N_IN4);
MOSFET_Motor heavyMotor(MOSFET_EN, MOSFET_PWM1, MOSFET_PWM2);
A4988_Stepper stepperMotor(A4988_STEP, A4988_DIR, A4988_EN);

// --- ISR ---
void isr_heavy()
{
  if (digitalRead(ENC_H_B))
    ticksHeavy++;
  else
    ticksHeavy--;
}

// --- SETUP ---
void setup()
{
  Serial.begin(115200);

  motorLeft.begin();
  motorRight.begin();
  heavyMotor.begin();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_H_A, INPUT_PULLUP); pinMode(ENC_H_B, INPUT_PULLUP);

  pinMode(LIMIT_TOP, INPUT_PULLUP);
  pinMode(LIMIT_BOTTOM, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_H_A), isr_heavy, RISING);
}

// --- LOOP ---
void loop()
{
  handlePacket();
  // updateArm();
}

// --- ENCODER READ ---
long getTicks()
{
  noInterrupts();
  long t = ticksHeavy;
  interrupts();
  return t;
}

// --- ARM CONTROL ---
void updateArm()
{
  bool topHit = !digitalRead(LIMIT_TOP);
  bool bottomHit = !digitalRead(LIMIT_BOTTOM);

  long current = getTicks();
  long error = armTarget - current;

  // dead zone
  if (abs(error) < 5)
  {
    heavyMotor.setPower(0);
    return;
  }

  // move up
  if (error > 0)
  {
    if (topHit)
    {
      heavyMotor.setPower(0);
      armTarget = current;
    }
    else
    {
      heavyMotor.setPower(140);
    }
  }

  // move down
  else
  {
    if (bottomHit)
    {
      heavyMotor.setPower(0);
      armTarget = current;
    }
    else
    {
      heavyMotor.setPower(-140);
    }
  }
}

// --- PACKET HANDLER ---
void handlePacket()
{
  if (!readPacket(Serial, pkt))
    return;

  switch (pkt.cmd)
  {
    case CMD_EUP:
      stepSize = pkt.b1;
      armTarget += stepSize;
      break;

    case CMD_EDOWN:
      stepSize = pkt.b1;
      armTarget -= stepSize;
      break;

    case CMD_STOP:
      heavyMotor.setPower(0);
      armTarget = getTicks();
      break;

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

    case CMD_ARM:
    {
      uint8_t dir = pkt.b1;

      bool topHit = !digitalRead(LIMIT_TOP);
      bool bottomHit = !digitalRead(LIMIT_BOTTOM);

      if (dir == A_CW) // UP (CW)
      {
          if (!topHit)
              heavyMotor.setPower(140);
          else
              heavyMotor.setPower(0);
      }
      else if (dir == A_CCW) // DOWN (CCW)
      {
          if (!bottomHit)
              heavyMotor.setPower(-140);
          else
              heavyMotor.setPower(0);
      }
      else
      {
          heavyMotor.setPower(0);
      }

      break;
    }

    case CMD_GROT:
    {
      int dir = pkt.b1;

      if (dir == G_CW)
      {
          // CW (R1)
          stepperMotor.setSpeed(1, 800);
      }
      else if (dir == G_CCW)
      {
          // CCW (R2)
          stepperMotor.setSpeed(-1, 800);
      }
      else
      {
          // STOP
          stepperMotor.setSpeed(0, 0);
      }

      break;
    }

    case CMD_RELAY:
    {
      digitalWrite(RLYL, pkt.b1 == RELAY_ON ? HIGH : LOW);
      digitalWrite(RLYR, pkt.b2 == RELAY_ON? HIGH : LOW);
      
      break;
    }

    default:
      break;
  }
}