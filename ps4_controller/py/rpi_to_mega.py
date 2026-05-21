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

# -------------------------
# Main loop
# -------------------------
while True:
    pygame.event.pump()

    ly = ds4.get_axis(1)
    lx = ds4.get_axis(0)

    # base movement
    base = int(-ly * 255)
    turn = int(lx * TURN_GAIN)

    left = base + turn
    right = base - turn

    # clamp
    left = max(-255, min(255, left))
    right = max(-255, min(255, right))

    # deadzone
    if abs(left) < DEADZONE:
        left = 0
    if abs(right) < DEADZONE:
        right = 0

    # change detection (IMPORTANT)
    if left == last_left and right == last_right:
        continue

    last_left = left
    last_right = right

    # direction + speed split
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

    time.sleep(0.02)