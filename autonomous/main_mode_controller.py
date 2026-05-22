import pygame
import time
import threading
import communications.protocol as pcl
from communications.serial_bridge import SerialBridge
from fsm import Phase1FSM
from robot import Robot

# -------------------------
# Serial + Robot setup
# -------------------------
serial = SerialBridge("/dev/ttyUSB0", 115200)
time.sleep(2)
print("Connected to Mega")

start_event = threading.Event()
robot = Robot(ip='', start_event=start_event)
fsm = Phase1FSM(robot)

# -------------------------
# Joystick setup
# -------------------------
pygame.init()
pygame.joystick.init()
ds4 = pygame.joystick.Joystick(0)
ds4.init()

# -------------------------
# Mode management
# -------------------------
MODES = ['MANUAL', 'AUTONOMOUS', 'DEBUG']
mode_index = 0  # start in MANUAL

autonomous_mode = False
fsm_pause_event = threading.Event()
fsm_pause_event.set()  # start paused

def get_mode():
    return MODES[mode_index]

# -------------------------
# FSM thread
# -------------------------
def fsm_thread():
    while True:
        fsm_pause_event.wait()
        if get_mode() != 'AUTONOMOUS':
            time.sleep(0.05)
            continue
        fsm.step()

t = threading.Thread(target=fsm_thread, daemon=True)
t.start()

# -------------------------
# Debug helpers
# -------------------------
debug_thread_running = False  # guard: only one debug function at a time

def run_with_timeout(fn, timeout=3.0):
    """Run a blocking function in a thread, stop motors after timeout."""
    global debug_thread_running
    if debug_thread_running:
        print("[DEBUG] Already running a function, wait for it to finish")
        return

    def wrapper():
        global debug_thread_running
        debug_thread_running = True
        print(f"[DEBUG] Running {fn.__name__} for {timeout}s...")

        result_holder = [None]
        def target():
            result_holder[0] = fn()

        fn_thread = threading.Thread(target=target, daemon=True)
        fn_thread.start()
        fn_thread.join(timeout=timeout)

        if fn_thread.is_alive():
            print(f"[DEBUG] Timeout reached — stopping motors")
        else:
            print(f"[DEBUG] {fn.__name__} finished — result: {result_holder[0]}")

        robot.drivetrain.stop()
        debug_thread_running = False

    threading.Thread(target=wrapper, daemon=True).start()

def print_pid():
    lf = robot.line_follow
    print(f"[PID] Kp={lf.Kp:.3f}  Ki={lf.Ki:.3f}  Kd={lf.Kd:.3f}")

# -------------------------
# Control tuning
# -------------------------
DEADZONE    = 20
PID_STEP_PD = 0.05
PID_STEP_I  = 0.01
DEBUG_TIMEOUT = 3.0

last_left     = None
last_right    = None
last_arm      = None
last_grip_rot = None
last_relay1   = None
last_relay2   = None

last_options  = False
last_cross    = False  # autonomous start

# Debug button edge tracking
last_dbg_x    = False  # follow_until_decision
last_dbg_o    = False  # creep_until_line
last_dbg_sq   = False  # fsm.step()
last_dbg_tr   = False  # align_to_cube
last_dup      = False  # Kp +
last_ddown    = False  # Kp -
last_dleft    = False  # Kd -
last_dright   = False  # Kd +
last_L1       = False  # Ki +
last_L2       = False  # Ki -
last_R1       = False  # print PID

arm_state    = None
grip_rot     = None
relay1_state = pcl.RELAY_OFF
relay2_state = pcl.RELAY_OFF

# -------------------------
# Main loop
# -------------------------
while True:
    pygame.event.pump()

    # ── Options — cycle modes ─────────────────────────────────
    options = ds4.get_button(9)
    if options and not last_options:
        # Clean up current mode before switching
        if get_mode() == 'AUTONOMOUS':
            fsm_pause_event.set()
            start_event.clear()
            serial.send_no_wait(pcl.build_packet(pcl.CMD_STOP, 0, 0, 0, 0))
        elif get_mode() == 'DEBUG':
            robot.drivetrain.stop()

        mode_index = (mode_index + 1) % len(MODES)

        if get_mode() == 'AUTONOMOUS':
            fsm_pause_event.clear()
            print("[MODE] AUTONOMOUS — press X to start FSM")
        elif get_mode() == 'DEBUG':
            print("[MODE] DEBUG")
            print("  X       = follow_until_decision (3s)")
            print("  Circle  = creep_until_line")
            print("  Square  = fsm.step()")
            print("  Triangle= align_to_cube (3s)")
            print("  D-Up/Dn = Kp +/-")
            print("  D-Lt/Rt = Kd +/-")
            print("  L1/L2   = Ki +/-")
            print("  R1      = print PID")
            print_pid()
        else:
            print("[MODE] MANUAL")
    last_options = options

    # ─────────────────────────────────────────────────────────
    # AUTONOMOUS MODE
    # ─────────────────────────────────────────────────────────
    if get_mode() == 'AUTONOMOUS':
        cross = ds4.get_button(0)
        if cross and not last_cross:
            start_event.set()
            print("[FSM] Started")
        last_cross = cross
        time.sleep(0.02)
        continue

    # ─────────────────────────────────────────────────────────
    # DEBUG MODE
    # ─────────────────────────────────────────────────────────
    if get_mode() == 'DEBUG':
        lf = robot.line_follow

        # ── Function triggers ─────────────────────────────────
        dbg_x  = ds4.get_button(0)
        dbg_o  = ds4.get_button(1)
        dbg_sq = ds4.get_button(2)
        dbg_tr = ds4.get_button(3)

        if dbg_x and not last_dbg_x:
            run_with_timeout(robot.line_follow.follow_until_decision, DEBUG_TIMEOUT)
        if dbg_o and not last_dbg_o:
            run_with_timeout(robot.line_follow.creep_until_line, DEBUG_TIMEOUT)
        if dbg_sq and not last_dbg_sq:
            print("[DEBUG] Running fsm.step()")
            threading.Thread(target=fsm.step, daemon=True).start()
        if dbg_tr and not last_dbg_tr:
            label = robot.scan_cube()
            if label:
                run_with_timeout(lambda: robot.align_to_cube(label), DEBUG_TIMEOUT)
            else:
                print("[DEBUG] No cube detected for align_to_cube")

        last_dbg_x  = dbg_x
        last_dbg_o  = dbg_o
        last_dbg_sq = dbg_sq
        last_dbg_tr = dbg_tr

        # ── PID tuning ────────────────────────────────────────
        dup    = ds4.get_hat(0)[1] ==  1   # D-pad up
        ddown  = ds4.get_hat(0)[1] == -1   # D-pad down
        dright = ds4.get_hat(0)[0] ==  1   # D-pad right
        dleft  = ds4.get_hat(0)[0] == -1   # D-pad left
        L1     = ds4.get_button(4)
        L2     = ds4.get_button(6)
        R1     = ds4.get_button(5)

        if dup    and not last_dup:    lf.Kp = round(lf.Kp + PID_STEP_PD, 3); print_pid()
        if ddown  and not last_ddown:  lf.Kp = round(lf.Kp - PID_STEP_PD, 3); print_pid()
        if dright and not last_dright: lf.Kd = round(lf.Kd + PID_STEP_PD, 3); print_pid()
        if dleft  and not last_dleft:  lf.Kd = round(lf.Kd - PID_STEP_PD, 3); print_pid()
        if L1     and not last_L1:     lf.Ki = round(lf.Ki + PID_STEP_I,  3); print_pid()
        if L2     and not last_L2:     lf.Ki = round(lf.Ki - PID_STEP_I,  3); print_pid()
        if R1     and not last_R1:     print_pid()

        last_dup    = dup
        last_ddown  = ddown
        last_dright = dright
        last_dleft  = dleft
        last_L1     = L1
        last_L2     = L2
        last_R1     = R1

        time.sleep(0.02)
        continue

    # ─────────────────────────────────────────────────────────
    # MANUAL MODE
    # ─────────────────────────────────────────────────────────

    ly = ds4.get_axis(1)
    lx = ds4.get_axis(0)

    stickY = int((ly + 1.0) * 127.5)
    stickX = int((lx + 1.0) * 127.5)

    drive = int(((255 - stickY) / 255.0) * 510 - 255)
    turn  = int(((255 - stickX) / 255.0) * 510 - 255)

    if abs(drive) < DEADZONE: drive = 0
    if abs(turn)  < DEADZONE: turn  = 0

    right = drive + turn
    left  = drive - turn
    left  = max(-255, min(255, left))
    right = max(-255, min(255, right))

    if left != last_left or right != last_right:
        last_left, last_right = left, right
        l_dir = pcl.MT_FWD if left  >= 0 else pcl.MT_RVS
        r_dir = pcl.MT_FWD if right >= 0 else pcl.MT_RVS
        serial.send_no_wait(pcl.build_packet(pcl.CMD_MOTORS, l_dir, abs(left), r_dir, abs(right)))

    L1_man = ds4.get_button(4)
    L2_man = ds4.get_button(6)
    arm_state = pcl.A_CW if (L1_man and not L2_man) else pcl.A_CCW if (L2_man and not L1_man) else pcl.A_STOP
    if arm_state != last_arm:
        serial.send_no_wait(pcl.build_packet(pcl.CMD_ARM, arm_state))
        last_arm = arm_state

    R1_man = ds4.get_button(5)
    R2_man = ds4.get_button(7)
    grip_rot = pcl.G_CW if (R1_man and not R2_man) else pcl.G_CCW if (R2_man and not R1_man) else pcl.G_STOP
    if grip_rot != last_grip_rot:
        serial.send_no_wait(pcl.build_packet(pcl.CMD_GROT, grip_rot))
        last_grip_rot = grip_rot

    cross_relay  = ds4.get_button(0)
    circle       = ds4.get_button(1)
    relay1_state = pcl.RELAY_ON if cross_relay else pcl.RELAY_OFF
    relay2_state = pcl.RELAY_ON if circle      else pcl.RELAY_OFF
    if relay1_state != last_relay1 or relay2_state != last_relay2:
        serial.send_no_wait(pcl.build_packet(pcl.CMD_RELAY, relay1_state, relay2_state))
        last_relay1, last_relay2 = relay1_state, relay2_state

    time.sleep(0.02)