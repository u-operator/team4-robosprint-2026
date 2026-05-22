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
  stepperMotor.begin();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_H_A, INPUT_PULLUP); pinMode(ENC_H_B, INPUT_PULLUP);

  pinMode(LIMIT_TOP1, INPUT_PULLUP);
  pinMode(LIMIT_TOP2, INPUT_PULLUP);

  pinMode(RLYL, OUTPUT);
  pinMode(RLYR, OUTPUT);
  digitalWrite(RLYL, HIGH);
  digitalWrite(RLYR, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENC_H_A), isr_heavy, RISING);
  stepperMotor.setSpeed(1, 800);
}

// --- LOOP ---
void loop()
{
  handlePacket();
  // updateArm();
  stepperMotor.update();
}

// --- ENCODER READ ---
long getTicks()
{
  noInterrupts();
  long t = ticksHeavy;
  interrupts();
  return t;
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

      bool top1Hit = !digitalRead(LIMIT_TOP1);
      bool top2Hit = !digitalRead(LIMIT_TOP2);

      if (dir == A_CW) // UP (CW)
      {
          if (!(top1Hit && top2Hit))
              heavyMotor.setPower(200);
      }
      else if (dir == A_CCW) // DOWN (CCW)
      {
              heavyMotor.setPower(-200);
      }
      else
      {
          heavyMotor.setPower(0);
      }

      break;
    }

    case CMD_GROT:
    {
      uint8_t dir = pkt.b1;

      if (dir == G_CW)
      {
          // CW (R1)
          stepperMotor.setSpeed(1, 300);
      }
      else if (dir == G_CCW)
      {
          // CCW (R2)
          stepperMotor.setSpeed(-1, 300);
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
      bool relay1 = (pkt.b1 == RELAY_ON);
      bool relay2 = (pkt.b2 == RELAY_ON);

      digitalWrite(RLYL, relay1 ? LOW : HIGH);
      digitalWrite(RLYR, relay2 ? LOW : HIGH);

      break;
    }
    

    default:
      break;
  }
}