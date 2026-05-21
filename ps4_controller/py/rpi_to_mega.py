import pygame
import time
import serial
import communications.protocol as pcl

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
time.sleep(2)

pygame.init()
pygame.joystick.init()

ds4 = pygame.joystick.Joystick(0)
ds4.init()

DEADZONE = 20

while True:
    pygame.event.pump()

    ly = ds4.get_axis(1)
    lx = ds4.get_axis(0)

    base = int(-ly * 255)
    turn = int(lx * 180)

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

    l_dir = pcl.MT_FWD if left >= 0 else pcl.MT_RVS
    r_dir = pcl.MT_FWD if right >= 0 else pcl.MT_RVS

    pkt = pcl.build_packet(
        pcl.CMD_MOTORS,
        l_dir,
        abs(left),
        r_dir,
        abs(right)
    )

    ser.write(pkt)

    time.sleep(0.05)