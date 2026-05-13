from enum import Enum, auto
import time
from idlelib.config import IdleConf


class State(Enum):
    IDLE = auto()
    NAVIGATE_TO_PICK = auto()
    SCAN_CUBE = auto()
    PICK_CUBE = auto()
    DISPLACE_FAKE = auto() # needed?
    NAVIGATE_TO_STORE = auto()
    DEPOSIT = auto()
    RETURN_TO_START = auto()

REAL_CUBES = {}
PHASE1_DURATION = 180 # 3minutes

class Phase1FSM:
    def __init__(self, robot):
        self.robot = robot
        self.state = State.IDLE
        self.detected_label = None
        self.start_time = None

    def time_remaining(self):
        return PHASE1_DURATION - (time.time() - self.start_time)

    def run(self):
        while True:

            if self.start_time and self.time_remaining() <= 0:
                self.state = State.RETURN_TO_START

            self.step()

            if self.state == State.RETURN_TO_START:
                self.robot.navigate_to("start_zone")
                break # Phase 1 complete

    def step(self):
        match self.state:

            case State.IDLE:
                self.robot.wait_to_start()
                self.start_time = time.time()
                self.state = State.NAVIGATE_TO_PICK

            case State.NAVIGATE_TO_PICK:
                success = self.robot.navigate("start zone", "pick zone")
                if success:
                    self.state = State.SCAN_CUBE
                else:
                    self.state = State.IDLE

            case State.SCAN_CUBE:
                if not self.robot.cubes_remaining:
                    self.state = State.RETURN_TO_START
                    return

                self.detected_label = self.robot.scan_cube()

                if self.detected_label in REAL_CUBES:
                    self.state = State.PICK_CUBE
                else:
                    self.state = State.DISPLACE_FAKE

            case State.PICK_CUBE:
                self.robot.pick_cube()
                self.state = State.NAVIGATE_TO_STORE

            case State.DISPLACE_FAKE:
                pass

            case State.NAVIGATE_TO_STORE:
                success = self.robot.navigate("pick zone", "storing zone")
                if success:
                    self.state = State.DEPOSIT

            case State.DEPOSIT:
                self.robot.deposit_cube()

                if self.time_remaining() > 10:
                    self.state = State.NAVIGATE_TO_PICK
                else:
                    self.state = State.RETURN_TO_START

