import communications.protocol as pcl
from communications.serial_bridge import SerialBridge

class Drivetrain:
    def __init__(self, serial: SerialBridge):
        self.serial = serial

    # ── Core ────────────────────────────────────────
    def set_motors(self, speed_left: int, speed_right: int):
        """
        speed: -255 to 255
        Negative = reverse, Positive = forward, 0 = stop
        Called by line_follower.py for PID correction.
        """
        l_dir = pcl.MT_FWD if speed_left > 0 else pcl.MT_RVS
        r_dir = pcl.MT_FWD if speed_right > 0 else pcl.MT_RVS
        speed_left = abs(speed_left)
        speed_right = abs(speed_right)

        self.serial.send_no_wait(pcl.build_packet(pcl.CMD_MOTORS, l_dir, speed_left, r_dir, speed_right))


    def stop(self):
        self.set_motors(0, 0)

    def cleanup(self):
        """Call this on program exit."""
        pass