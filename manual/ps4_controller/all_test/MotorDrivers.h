#ifndef MOTOR_DRIVERS_H
#define MOTOR_DRIVERS_H

#include <Arduino.h>

// Class for the L298N Dual Motor Driver (Now tracking Encoders)
class L298N_Motor {
  public:
    L298N_Motor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin);
    void begin();
    void setPower(int power); 
    void stop();

  private:
    uint8_t _en, _in1, _in2;
};

// Class for the MOSFET DC Motor Driver (With PID)
class MOSFET_Motor {
  public:
    MOSFET_Motor(uint8_t enPin, uint8_t pwm1Pin, uint8_t pwm2Pin);
    void begin();
    void setPower(int power); 
    void setPIDConstants(float kp, float ki, float kd);
    void setTargetSpeed(float targetTicksPerSec);
    void computePID(long currentEncoderTicks); 

  private:
    uint8_t _en, _pwm1, _pwm2;
    float _kp = 1.0, _ki = 0.0, _kd = 0.0;
    float _targetSpeed = 0.0;
    long _lastTicks = 0;
    unsigned long _lastTime = 0;
    float _integralError = 0.0;
    float _lastError = 0.0;
};

// Class for the A4988 Stepper Driver (Non-blocking)
class A4988_Stepper {
  public:
    A4988_Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enPin);
    void begin();
    void enable(bool state);
    void move(long steps, int delayMicros);
    void setSpeed(int dir, int delayMicros);
    void update(); 

  private:
    uint8_t _step, _dir, _en;
    long _stepsRemaining = 0;
    unsigned long _lastStepTime = 0;
    int _stepDelay = 500;
    bool _pulseState = LOW;
    long _direction = 0; 
};

#endif