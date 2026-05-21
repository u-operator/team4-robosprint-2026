import serial
import time
import communications.protocol as pcl

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
time.sleep(2)

print("connected")

while True:

    pkt = pcl.build_packet(
        pcl.CMD_MOTORS,
        pcl.MT_FWD,
        120,
        pcl.MT_FWD,
        120
    )

    print(list(pkt))

    ser.write(pkt)

    time.sleep(1)