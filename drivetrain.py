import RPi.GPIO as GPIO

class Drivetrain:
    def __init__(self):
        # Motor driver pins (adjust to your wiring)
        self.LEFT_PWM  = 12   # PWM pin for left motor speed
        self.LEFT_DIR  = 16   # Direction pin for left motor
        self.RIGHT_PWM = 13   # PWM pin for right motor speed
        self.RIGHT_DIR = 20   # Direction pin for right motor

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LEFT_PWM,  GPIO.OUT)
        GPIO.setup(self.LEFT_DIR,  GPIO.OUT)
        GPIO.setup(self.RIGHT_PWM, GPIO.OUT)
        GPIO.setup(self.RIGHT_DIR, GPIO.OUT)

        # PWM at 1000Hz
        self.pwm_left  = GPIO.PWM(self.LEFT_PWM,  1000)
        self.pwm_right = GPIO.PWM(self.RIGHT_PWM, 1000)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

    # ── Core ────────────────────────────────────────
    def set_motors(self, speed_left: float, speed_right: float):
        """
        speed: -100 to 100
        Negative = reverse, Positive = forward, 0 = stop
        Called by line_follower.py for PID correction.
        """
        self._drive(self.pwm_left,  self.LEFT_DIR,  speed_left)
        self._drive(self.pwm_right, self.RIGHT_DIR, speed_right)

    def stop(self):
        self.set_motors(0, 0)

    def cleanup(self):
        """Call this on program exit."""
        self.stop()
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()

    # ── Internal ─────────────────────────────────────
    def _drive(self, pwm, dir_pin, speed: float):
        speed = max(-100, min(100, speed))   # clamp to valid range
        GPIO.output(dir_pin, GPIO.HIGH if speed >= 0 else GPIO.LOW)
        pwm.ChangeDutyCycle(abs(speed))