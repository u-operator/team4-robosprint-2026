from camera import Camera
from drivetrain import Drivetrain
from arm import Arm
from line_follow import LineFollower


class Robot:
    def __init__(self, ip: str):
        # Initialize hardware
        self.camera = Camera(ip=ip)
        self.drivetrain = Drivetrain()
        self.arm = Arm()
        self.has_cube = False
        self.line_follow = LineFollower(self.camera, self.drivetrain)

    # Nav
    def navigate(self, origin: str, dest: str) -> bool:
        """
            Move from origin to dest, making correct decisions at junctions.
            Returns True on success. False on any execution error or raise an error

            Zones: 'pick_zone', 'storing_zone', 'start_zone'
            """


        # --- Routes that pass through a junction or curves
        # Decisions to take when junction or curve detected
        routes = {
            ("pick_zone", "storing_zone"): ['left', 'left'],
            ("storing_zone", "pick_zone"): ['right', 'right'],
            ('start_zone', 'pick_zone'): ['left']
        }

        if (origin, dest) in routes:
            decisions = routes[(origin, dest)]
            decision_cnt = 0
            for decision in decisions:
                reached_junction = self.line_follow.follow()
                if reached_junction:
                    self.turn(decision[decision_cnt])
                elif reached_junction is None: # LINE LOST
                    return False
        else:
            raise ValueError(f"No known route from '{origin}' to '{dest}'")

    def turn(self, direction: str):
        if direction == 'left':
            self.drivetrain.set_motors()
        if direction == 'right':
            self.drivetrain.set_motors()

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
        self.line_follow.reset_pid()

    def start_button_pressed(self) -> bool:
        # Read from pin
        return False

