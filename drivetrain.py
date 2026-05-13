import RPi.GPIO as GPIO

class Drivetrain:
    def __init__(self):
        # Setup your motor GPIO pins
        self.left_motor  = ...
        self.right_motor = ...

        # IR sensors or camera line detection
        self.line_sensor_left  = ...
        self.line_sensor_right = ...

    # --- Navigation ---
    def move_to(self, x, y) -> bool:
        """High-level move, uses line following internally."""
        self.follow_line_until(target=(x, y))
        return True

    # --- Line Following ---
    def follow_line_until(self, target):
        """Follow line with motor correction until target reached."""
        while not self.at_target(target):
            self.correct_motors()

    def correct_motors(self):
        """Read sensors, apply correction."""
        left  = self.read_sensor(self.line_sensor_left)
        right = self.read_sensor(self.line_sensor_right)

        if left and not right:
            self.turn_slightly_right()
        elif right and not left:
            self.turn_slightly_left()
        else:
            self.go_straight()

    # --- Motor Primitives ---
    def go_straight(self):
        self.set_motors(speed_left=100, speed_right=100)

    def turn_slightly_left(self):
        self.set_motors(speed_left=60, speed_right=100)

    def turn_slightly_right(self):
        self.set_motors(speed_left=100, speed_right=60)

    def stop(self):
        self.set_motors(0, 0)

    def set_motors(self, speed_left, speed_right):
        # Write to GPIO / PWM here
        pass

    def read_sensor(self, pin) -> bool:
        return GPIO.input(pin)

    def at_target(self, target) -> bool:
        # Use encoders, IMU, or junction counting
        pass