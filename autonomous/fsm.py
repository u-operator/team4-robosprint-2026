from enum import Enum, auto
import time


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

REAL_CUBES = ['B', 'C', 'E', 'M', 'R', 'U']
PHASE1_DURATION = 180 # 3minutes


class Phase1FSM:
    def __init__(self, robot):
        self.robot = robot
        self.cur_state = State.WAIT_TO_START
        self.prev_state = None
        self.detected_label = None
        self.start_time = None

        # Set before every GO_TO_LINE / NAVIGATE_TO_* transition
        self.nav_origin = None
        self.nav_dest = None

    def transition(self, new_state):
        """Update cur/prev state. Always use this instead of setting cur_state directly."""
        self.prev_state = self.cur_state
        self.cur_state = new_state

    def time_remaining(self):
        return PHASE1_DURATION - (time.time() - self.start_time)

    def _set_nav(self, origin: str, dest: str, next_state: State):
        """Convenience: set nav context then transition."""
        self.nav_origin = origin
        self.nav_dest   = dest
        self.transition(next_state)



    # Main Loop
    def run(self):
        while True:

            if self.start_time and self.time_remaining() <= 0:
                self.transition(State.RETURN_TO_START)

    def step(self):
        match self.cur_state:
            case State.WAIT_TO_START:
                self.robot.wait_to_start()
                self.start_time = time.time()
                self._set_nav("start_zone", "pick_zone", State.GO_TO_LINE)
                
            case State.IDLE: # When waiting to reattempt, this is the state
                self.robot.wait_to_start()
                self._set_nav("start_zone", "pick_zone", State.GO_TO_LINE)

            case State.NAVIGATE_TO_PICK:
                success = self.robot.navigate(self.nav_origin, self.nav_dest)
                if success:
                    self.transition(State.SCAN_CUBE)
                else:
                    self.transition(State.IDLE)

            case State.NAVIGATE_TO_STORE:
                success = self.robot.navigate(self.nav_origin, self.nav_dest)
                if success:
                    self.transition(State.DEPOSIT)
                else:
                    self.transition(State.IDLE)

            case State.SCAN_CUBE:
                if not self.robot.cubes_remaining:
                    self.transition(State.RETURN_TO_START)
                    return

                self.detected_label = self.robot.scan_cube()

                if self.detected_label in REAL_CUBES:
                    self.transition(State.PICK_CUBE)
                else:
                    self.transition(State.DISPLACE_FAKE)

            case State.PICK_CUBE:
                success = self.robot.pick_cube()
                if success:
                    self._set_nav("pick_zone", "storing_zone", State.NAVIGATE_TO_STORE)
                else:
                    # Pick failed — re-scan
                    self.transition(State.SCAN_CUBE)

            case State.DISPLACE_FAKE:
                pass

            case State.DEPOSIT:
                self.robot.deposit_cube()

                if self.time_remaining() > 10:
                    self._set_nav("storing_zone", "pick_zone", State.NAVIGATE_TO_PICK)
                else:
                    self.transition(State.RETURN_TO_START)

            case State.GO_TO_LINE: # Intermediate state between navigating the field
                match self.prev_state:
                    case State.IDLE | State.IDLE.WAIT_TO_START:
                        if not self.robot.line_follow.creep_until_line():
                            self.cur_state = State.IDLE
                        self.cur_state = State.NAVIGATE_TO_PICK

                    case State.DEPOSIT:
                        pass
                        # TODO: Figure out how to return the robot to the line
                        # self.cur_state = State.NAVIGATE_TO_PICK

                    case State.PICK_CUBE:
                        pass
                        # TODO: Same here
                        # self.cur_state = State.NAVIGATE_TO_STORE