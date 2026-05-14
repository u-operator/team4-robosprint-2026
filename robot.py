from camera import Camera
from drivetrain import Drivetrain
from arm import Arm
from line_follow import LineFollower
from communications.serial_bridge import SerialBridge

REAL_CUBES = ['B', 'C', 'E', 'M', 'R', 'U']

# TODO: Test all functions

class Robot:
    def __init__(self, ip: str):
        # Initialize hardware
        self.s_com = SerialBridge()
        self.camera = Camera(ip=ip)
        self.drivetrain = Drivetrain(self.s_com)
        self.arm = Arm(self.s_com)
        self.has_cube = False
        self.line_follow = LineFollower(self.camera, self.drivetrain)


    # Movement
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
        """
        Turn
        """
        if direction == 'left':
            self.drivetrain.set_motors(0, 100)
        if direction == 'right':
            self.drivetrain.set_motors(100, 0)

    def align_to_cube(self, target_label: str) -> bool:
        """
        Visually center the robot on the target cube.
        Nudges motors until cube is horizontally centered.
        Returns True when aligned, False if cube lost.
        """
        ALIGN_THRESHOLD = 20  # px — acceptable offset from center
        MAX_ATTEMPTS = 50  # give up after this many frames
        Kp = 0.3 # For error correction

        for _ in range(MAX_ATTEMPTS):
            frame = self.camera.capture()
            h, w = frame.shape[:2]
            cx, cy = self.camera.find_cube(frame, target_label)  # ← you implement this

            if cx is None:
                # Cube not visible — stop and fail
                self.drivetrain.stop()
                return False

            error = cx - (w // 2)  # + = cube is right, - = cube is left

            if abs(error) < ALIGN_THRESHOLD:
                self.drivetrain.stop()
                return True  # aligned!

            # Nudge: turn toward the cube
            correction = int(Kp * error)  # small P-only gain
            self.drivetrain.set_motors(
                speed_left=correction,  # one side forward
                speed_right=-correction  # other side backward = pivot turn
            )

        self.drivetrain.stop()
        return False  # failed to align

    def approach_cube(self, target_label: str) -> bool:

        """
        Drive slowly forward until cube fills enough of the frame (close enough).
        Returns True when in pick range.
        """
        MIN_BOX_HEIGHT = 80  # px — tune based on your camera + arm reach
        MAX_ATTEMPTS = 60

        for _ in range(MAX_ATTEMPTS):
            frame = self.camera.capture()
            box = self.camera.get_cube_box(frame, target_label)  # (x, y, w, h)

            if box is None:
                self.drivetrain.stop()
                return False

            _, _, bw, bh = box

            if bh >= MIN_BOX_HEIGHT:
                self.drivetrain.stop()
                return True  # close enough to pick

            self.drivetrain.set_motors(40, 40)  # crawl forward

        self.drivetrain.stop()
        return False

    # Vision
    def scan_cube(self) -> str:

        frame = self.camera.capture()
        label = self.camera.find_best_cube(frame, REAL_CUBES)
        return label

    # Arm
    def pick_cube(self):
        """
            Scan for available cubes, align, approach then pick up the cube
            Returns True on success. False if fail to align or approach
        """
        label = self.scan_cube()

        if not self.align_to_cube(label):
            return False
        if not self.approach_cube(label):
            return False

        self.arm.lower_arm()
        self.arm.close_grip()
        self.arm.raise_arm()
        self.has_cube = True
        return True

    def deposit_cube(self):

        self.arm.lower_arm()
        self.arm.release_grip()
        self.arm.raise_arm()
        self.has_cube = False

    # Signals
    def wait_to_start(self):

        while not self.start_button_pressed():
            pass
        self.line_follow.reset_pid()

    def start_button_pressed(self) -> bool:
        # Read from pin
        return False

