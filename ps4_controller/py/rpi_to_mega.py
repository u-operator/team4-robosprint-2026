import pygame
import time
import communications.protocol as pcl
from communications.serial_bridge import SerialBridge

# -------------------------
# Serial setup
# -------------------------
serial = SerialBridge("/dev/ttyUSB0", 115200)
time.sleep(2)

print("Connected to Mega")

# -------------------------
# Joystick setup
# -------------------------
pygame.init()
pygame.joystick.init()

ds4 = pygame.joystick.Joystick(0)
ds4.init()

# -------------------------
# Control tuning
# -------------------------
DEADZONE = 20
TURN_GAIN = 140

last_left = None
last_right = None
last_arm = None
last_grip_rot = None
last_square = False
last_circle = False
last_relay1 = None
last_relay2 = None

arm_state = None
grip_rot = None
relay1_state = pcl.RELAY_OFF
relay2_state = pcl.RELAY_OFF

# -------------------------
# Main loop
# -------------------------
while True:
    pygame.event.pump()

    ly = ds4.get_axis(1)  # -1.0 to 1.0
    lx = ds4.get_axis(0)  # -1.0 to 1.0

    # ------------------------------------------------
    # Convert pygame axis -> PS2 style 0-255
    # ------------------------------------------------
    stickY = int((ly + 1.0) * 127.5)
    stickX = int((lx + 1.0) * 127.5)

    # ------------------------------------------------
    # Match Arduino map()
    # map(0,255,255,-255)
    # ------------------------------------------------
    drive = int(((255 - stickY) / 255.0) * 510 - 255)
    turn = int(((255 - stickX) / 255.0) * 510 - 255)

    # ------------------------------------------------
    # DEADZONE
    # ------------------------------------------------
    if abs(drive) < DEADZONE:
        drive = 0

    if abs(turn) < DEADZONE:
        turn = 0

    # ------------------------------------------------
    # MIXING
    # ------------------------------------------------
    left = drive + turn
    right = drive - turn

    # ------------------------------------------------
    # CLAMP
    # ------------------------------------------------
    left = max(-255, min(255, left))
    right = max(-255, min(255, right))

    # ------------------------------------------------
    # CHANGE DETECTION
    # ------------------------------------------------
    if left != last_left or right != last_right:
        last_left = left
        last_right = right

        l_dir = pcl.MT_FWD if left >= 0 else pcl.MT_RVS
        r_dir = pcl.MT_FWD if right >= 0 else pcl.MT_RVS

        packet = pcl.build_packet(
            pcl.CMD_MOTORS,
            l_dir,
            abs(left),
            r_dir,
            abs(right)
        )

        serial.send_no_wait(packet)

    # Arm elevator control
    L1 = ds4.get_button(4)
    L2 = ds4.get_button(6)

    if L1 and not L2:
        arm_state = pcl.A_CW  # UP
    elif L2 and not L1:
        arm_state = pcl.A_CCW  # DOWN
    else:
        arm_state = pcl.A_STOP  # STOP

        # send arm only if changed
    if arm_state != last_arm:
        arm_pkt = pcl.build_packet(
            pcl.CMD_ARM,
            arm_state
        )

        serial.send_no_wait(arm_pkt)
        last_arm = arm_state

    # Stepper motor, grip rotation control
    R1 = ds4.get_button(5)  # R1
    R2 = ds4.get_button(7)  # R2

    if R1 and not R2:
        grip_rot = pcl.G_CW  # CW
    elif R2 and not R1:
        grip_rot = pcl.G_CCW  # CCW
    else:
        grip_rot = pcl.G_STOP  # STOP

    if grip_rot != last_grip_rot:
        pkt = pcl.build_packet(
            pcl.CMD_GROT,
            grip_rot
        )

        serial.send_no_wait(pkt)
        last_grip_rot = grip_rot

    # Activating relay
    square = ds4.get_button(0)
    circle = ds4.get_button(1)

    # -------------------------
    # RELAY 1 (Square)
    # -------------------------
    if square and not last_square:
        relay1_state = pcl.RELAY_ON

    if not square and last_square:
        relay1_state = pcl.RELAY_OFF

    # -------------------------
    # RELAY 2 (Circle)
    # -------------------------
    if circle and not last_circle:
        relay2_state = pcl.RELAY_ON

    if not circle and last_circle:
        relay2_state = pcl.RELAY_OFF

    last_square = square
    last_circle = circle

    if relay1_state != last_relay1 or relay2_state != last_relay2:
        pkt = pcl.build_packet(pcl.CMD_RELAY, relay1_state, relay2_state)

        serial.send_no_wait(pkt)

        last_relay1 = relay1_state
        last_relay2 = relay2_state

    time.sleep(0.02)