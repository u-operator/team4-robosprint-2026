from communications.serial_bridge import SerialBridge
import communications.protocol as pcl

class Arm:

    # # ── Tunable constants ────────────────────────────────────────────────────
    # LIFT_SPEED          = 70    # PWM duty cycle 0-100 for elevator motor
    # LOWER_TICKS         = 400   # encoder ticks to reach pick height
    # RAISE_TICKS         = 400   # ticks to return to top (should mirror LOWER)
    # GRIPPER_CLOSE_STEPS = 200   # stepper steps to close gripper
    # GRIPPER_OPEN_STEPS  = 200   # stepper steps to open gripper
    # STEP_DELAY          = 0.001 # seconds between stepper pulses (controls speed)

    def __init__(self, serial: SerialBridge):
        # RPI is not connected directly to the drivers
        # Instead will send commands
        self.serial = serial

    # ── Public API (called by robot.py) ─────────────────────────────────────

    def lower_arm(self):
        self.serial.send(pcl.build_packet(pcl.CMD_ARM, pcl.A_DOWN, 0, 0, 0))

    def raise_arm(self):
        """Raise the elevator back"""
        self.serial.send(pcl.build_packet(pcl.CMD_ARM, pcl.A_UP, 0, 0, 0))


    def close_grip(self):
        """Close the gripper to grab a cube."""
        self.serial.send(pcl.build_packet(pcl.CMD_GRIP, pcl.G_CLOSE, 0, 0, 0))


    def release_grip(self):
        """Open the gripper to release a cube."""
        self.serial.send(pcl.build_packet(pcl.CMD_GRIP, pcl.G_OPEN, 0, 0, 0))

    def cleanup(self):
       pass
