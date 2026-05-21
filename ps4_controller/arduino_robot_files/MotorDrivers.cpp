#include "MotorDrivers.h"

// ==========================================
// L298N DC MOTOR IMPLEMENTATION
// ==========================================
L298N_Motor::L298N_Motor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin) {
  _en = enPin; _in1 = in1Pin; _in2 = in2Pin;
}

void L298N_Motor::begin() {
  pinMode(_en, OUTPUT); pinMode(_in1, OUTPUT); pinMode(_in2, OUTPUT);
  stop();
}

void L298N_Motor::setPower(int power) {
  power = constrain(power, -255, 255);
  if (power == 0) {
    stop();
  } else if (power > 0) {
    digitalWrite(_in1, HIGH); digitalWrite(_in2, LOW);
    analogWrite(_en, power);
  } else {
    digitalWrite(_in1, LOW); digitalWrite(_in2, HIGH);
    analogWrite(_en, abs(power));
  }
}

void L298N_Motor::stop() {
  digitalWrite(_in1, LOW); digitalWrite(_in2, LOW);
  analogWrite(_en, 0);
}

// ==========================================
// MOSFET DC MOTOR IMPLEMENTATION (WITH PID)
// ==========================================
MOSFET_Motor::MOSFET_Motor(uint8_t enPin, uint8_t pwm1Pin, uint8_t pwm2Pin) {
  _en = enPin; _pwm1 = pwm1Pin; _pwm2 = pwm2Pin;
}

void MOSFET_Motor::begin() {
  pinMode(_en, OUTPUT); pinMode(_pwm1, OUTPUT); pinMode(_pwm2, OUTPUT);
  digitalWrite(_en, HIGH); 
  setPower(0);
  _lastTime = millis();
}

void MOSFET_Motor::setPower(int power) {
  power = constrain(power, -255, 255);
  if (power == 0) {
    analogWrite(_pwm1, 0); analogWrite(_pwm2, 0);
  } else if (power > 0) {
    analogWrite(_pwm1, power); analogWrite(_pwm2, 0);
  } else {
    analogWrite(_pwm1, 0); analogWrite(_pwm2, abs(power));
  }
}

void MOSFET_Motor::setPIDConstants(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
  _integralError = 0; 
}

void MOSFET_Motor::setTargetSpeed(float targetTicksPerSec) {
  _targetSpeed = targetTicksPerSec;
}

void MOSFET_Motor::computePID(long currentEncoderTicks) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - _lastTime) / 1000.0; 
  if (deltaTime <= 0.001) return; 

  float currentSpeed = (currentEncoderTicks - _lastTicks) / deltaTime;
  float error = _targetSpeed - currentSpeed;
  _integralError += (error * deltaTime);
  float derivative = (error - _lastError) / deltaTime;
  
  float output = (_kp * error) + (_ki * _integralError) + (_kd * derivative);
  setPower((int)output);
  
  _lastTicks = currentEncoderTicks;
  _lastTime = currentTime;
  _lastError = error;
}

// ==========================================
// A4988 STEPPER MOTOR IMPLEMENTATION
// ==========================================
A4988_Stepper::A4988_Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin) {
  _step = stepPin; _dir = dirPin; _en = enPin;
}

void A4988_Stepper::begin() {
  pinMode(_step, OUTPUT); pinMode(_dir, OUTPUT); pinMode(_en, OUTPUT);
  enable(true);
}

void A4988_Stepper::enable(bool state) {
  digitalWrite(_en, state ? LOW : HIGH); // A4988: EN LOW = enabled
}

void A4988_Stepper::move(long steps, int delayMicros) {
  if (steps > 0) {
    digitalWrite(_dir, HIGH);
    _stepsRemaining = steps;
  } else {
    digitalWrite(_dir, LOW);
    _stepsRemaining = abs(steps);
  }
  _stepDelay = delayMicros;
}

void A4988_Stepper::setSpeed(int dir, int delayMicros)
{
  _direction = dir;      // 1 or -1 or 0
  _stepDelay = delayMicros;
}

void A4988_Stepper::update() void A4988_Stepper::update()
{
  if (_direction == 0) return;

  unsigned long currentMicros = micros();

  if (currentMicros - _lastStepTime < _stepDelay)
    return;

  _lastStepTime = currentMicros;

  // direction
  if (_direction > 0)
    digitalWrite(_dir, HIGH);   // CW
  else
    digitalWrite(_dir, LOW);    // CCW

  // step pulse
  digitalWrite(_step, HIGH);
  delayMicroseconds(2);
  digitalWrite(_step, LOW);
}