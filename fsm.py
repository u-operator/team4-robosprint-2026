from enum import Enum, auto
import time
from idlelib.config import IdleConf


class State(Enum):
    GO_TO_LINE = auto()
    WAIT_TO_START = auto()
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
        self.cur_state = State.WAIT_TO_START
        self.prev_state = None
        self.detected_label = None
        self.start_time = None

    def time_remaining(self):
        return PHASE1_DURATION - (time.time() - self.start_time)

    def run(self):
        attempt = 1
        while True:

            if self.start_time and self.time_remaining() <= 0:
                self.cur_state = State.RETURN_TO_START
                
            

    def step(self):
        match self.cur_state:
            case State.WAIT_TO_START: # Beginning state or first attempt
                self.robot.wait_to_start()
                self.start_time = time.time()
                self.cur_state = State.NAVIGATE_TO_PICK
                
            case State.IDLE: # When waiting to reattempt, this is the state
                self.robot.wait_to_start()
                self.cur_state = State.NAVIGATE_TO_PICK

            case State.NAVIGATE_TO_PICK:
                if self.cur_state is State.IDLE or State.WAIT_TO_START:  # Nav to pick from start
                   success = self.robot.navigate("start zone", "pick zone")
                elif self.prev_state is State.DEPOSIT:
                   success = self.robot.navigate('storing zone', 'pick zone') # Nav to pick from store zone
                if success:
                    self.cur_state = State.SCAN_CUBE
                else:
                    self.cur_state = State.IDLE

            case State.SCAN_CUBE:
                if not self.robot.cubes_remaining:
                    self.cur_state = State.RETURN_TO_START
                    return

                self.detected_label = self.robot.scan_cube()

                if self.detected_label in REAL_CUBES:
                    self.cur_state = State.PICK_CUBE
                else:
                    self.cur_state = State.DISPLACE_FAKE

            case State.PICK_CUBE:
                self.robot.pick_cube()
                self.cur_state = State.NAVIGATE_TO_STORE

            case State.DISPLACE_FAKE:
                pass

            case State.NAVIGATE_TO_STORE:
                success = self.robot.navigate("pick zone", "storing zone")
                if success:
                    self.cur_state = State.DEPOSIT
                else:
                    self.cur_state = State.IDLE

            case State.DEPOSIT:
                self.robot.deposit_cube()

                if self.time_remaining() > 10:
                    self.cur_state = State.NAVIGATE_TO_PICK
                else:
                    self.cur_state = State.RETURN_TO_START

            case State.GO_TO_LINE: # Intermediate state between navigating the field
                match self.prev_state:
                    case State.IDLE | State.IDLE.WAIT_TO_START:
                        if not self.robot.line_follow.creep_until_line():
                            self.cur_state = State.IDLE
                        self.cur_state = State.NAVIGATE_TO_PICK

                    case State.DEPOSIT:
                        pass
                        # self.cur_state = State.NAVIGATE_TO_PICK

                    case State.PICK_CUBE:
                        pass
                        # self.cur_state = State.NAVIGATE_TO_STORE