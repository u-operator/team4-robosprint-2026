# Constants and functions regarding the format of data to send
HEADER    = 0xFF
CMD_SERVO = 0x03
CMD_GRIP  = 0x04

def build_packet(cmd: int, b1: int, b2: int) -> bytes:
    checksum = (HEADER + cmd + b1 + b2) % 256
    return bytes([HEADER, cmd, b1, b2, checksum])