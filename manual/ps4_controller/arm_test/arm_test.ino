#include "pin_config.h"
#include "protocol.h"
#include "MotorDrivers.h"
#include "RobotActions.h"

void updateArm();
long getTicks();
void handlePacket();

volatile long ticksHeavy = 0;
Packet pkt;
long armTarget = 0;
int stepSize = 50;


void isr_heavy()
{
  if (digitalRead(ENC_H_B))
    ticksHeavy++;
  else
    ticksHeavy--;
}

void setup()
{
  Serial.begin(115200);

  heavyMotor.begin();

  pinMode(ENC_H_A, INPUT_PULLUP);
  pinMode(ENC_H_B, INPUT_PULLUP);

  pinMode(LIMIT_TOP, INPUT_PULLUP);
  pinMode(LIMIT_BOTTOM, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_H_A), isr_heavy, RISING);

  Serial.println("ARM TEST READY");
}

void loop()
{
  handlePacket();
  updateArm();

  // -------------------------
  // DEBUG OUTPUT
  // -------------------------
  static unsigned long t = 0;

  if (millis() - t > 200)
  {
    t = millis();

    Serial.print("Ticks:");
    Serial.print(getTicks());

    Serial.print(" Target:");
    Serial.print(armTarget);

    Serial.print(" Top:");
    Serial.print(!digitalRead(LIMIT_TOP));

    Serial.print(" Bottom:");
    Serial.println(!digitalRead(LIMIT_BOTTOM));
  }
}


long getTicks()
{
  noInterrupts();
  long t = ticksHeavy;
  interrupts();
  return t;
}

void updateArm()
{
  bool topHit = !digitalRead(LIMIT_TOP);
  bool bottomHit = !digitalRead(LIMIT_BOTTOM);

  long current = getTicks();
  long error = armTarget - current;

  // -------------------------
  // DEAD ZONE
  // -------------------------
  if (abs(error) < 5)
  {
    heavyMotor.setPower(0);
    return;
  }

  // -------------------------
  // MOVE UP
  // -------------------------
  if (error > 0)
  {
    if (topHit)
    {
      heavyMotor.setPower(0);
      armTarget = current; // clamp
    }
    else
    {
      heavyMotor.setPower(140);
    }
  }
}

void handlePacket()
{
  if (!readPacket(Serial, pkt))
    return;

  switch (pkt.cmd)
  {
    // -------------------------
    // STEP UP
    // -------------------------
    case CMD_EUP:
    {
      stepSize = pkt.b1;
      armTarget += stepSize;
      Serial.print("STEP UP → ");
      Serial.println(armTarget);
      break;
    }

    // -------------------------
    // STEP DOWN
    // -------------------------
    case CMD_EDOWN:
    {
      stepSize = pkt.b1;
      armTarget -= stepSize;
      Serial.print("STEP DOWN → ");
      Serial.println(armTarget);
      break;
    }

    // -------------------------
    // STOP ALL
    // -------------------------
    case CMD_STOP:
    {
      heavyMotor.setPower(0);
      Serial.println("STOP");
      break;
    }

    default:
      break;
  }
}