import pygame
import threading
import communications.protocol as pcl
from communications.serial_bridge import SerialBridge
import time

serial = SerialBridge()
def read_serial():
    while True:
        try:

            line = serial.ser.readline().decode(errors="ignore").strip()
            if line:
                print("[MEGA]", line)
        except:
            pass

threading.Thread(target=read_serial, daemon=True).start()

pygame.init()
pygame.joystick.init()
ds4 = pygame.joystick.Joystick(0)
ds4.init()

prev_up = False
prev_down = False

STEP_SIZE = 50

while True:
    pygame.event.pump()

    lx = ds4.get_axis(0)  # -1.0 to 1.0
    ly = ds4.get_axis(1)
    hat_x, hat_y = ds4.get_hat(0)

    # Map stick to motor speeds (-255 to 255)
    left_speed  = int(-ly * 255)
    right_speed = int(-ly * 255)

    # Add turning from left stick X
    turn = int(lx * 255)
    left_speed  = left_speed + turn
    right_speed = right_speed - turn

    # Clamp
    left_speed  = max(-255, min(255, left_speed))
    right_speed = max(-255, min(255, right_speed))

    l_dir = pcl.MT_FWD if left_speed > 0 else pcl.MT_RVS
    r_dir = pcl.MT_FWD if right_speed > 0 else pcl.MT_RVS

    print("l:", left_speed, "r:", right_speed)


    # Send to MEGA
    packet = pcl.build_packet(pcl.CMD_MOTORS, l_dir, abs(left_speed), r_dir, abs(right_speed))
    serial.send_no_wait(packet)

    if hat_y == 1:
        serial.send_no_wait(pcl.build_packet(pcl.CMD_EUP, STEP_SIZE))  # E_UP + step size

    if hat_y == -1:
        serial.send_no_wait(pcl.build_packet(pcl.CMD_EDOWN, STEP_SIZE))


    time.sleep(0.1)



