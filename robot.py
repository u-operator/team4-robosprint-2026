from camera import Camera
from drivetrain import Drivetrain
from arm import Arm

class Robot:
    def __init__(self, ip: str):
        # Initialize hardware
        self.camera = Camera(ip=ip)
        self.drivetrain = Drivetrain()
        self.arm = Arm()
        self.has_cube = False

    # Nav
    def navigate(self, origin: str, dest: str) -> bool:

        pass

    # Vision
    def scan_cube(self) -> str:

        frame = self.camera.capture()
        label = self.camera.read_label(frame)
        return label

    # Arm
    def pick_cube(self):
        self.arm.lower()
        self.arm.grip()
        self.arm.raise_arm()
        self.has_cube = True

    def deposit_cube(self):
        pass

    def place_at(self, location: str):
        pass

    # Signals
    def wait_to_start(self):

        while not self.start_button_pressed():
            pass

    def start_button_pressed(self) -> bool:
        # Read from pin
        return False

