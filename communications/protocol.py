# protocol.py
#
# ── Packet Format ────────────────────────────────────────────────────────────
#
#   All packets are fixed 7 bytes:
#   [HEADER] [CMD] [B1] [B2] [B3] [B4] [CHECKSUM]
#
#   HEADER   : 0xFF always
#   CMD      : command ID (see below)
#   B1–B4    : command-specific data bytes (unused bytes set to 0x00)
#   CHECKSUM : (HEADER + CMD + B1 + B2 + B3 + B4) % 256
#
# ── Command Table ────────────────────────────────────────────────────────────
#
#   CMD   Name        B1          B2            B3          B4
#   0x01  MOTORS      left_dir    left_speed    right_dir   right_speed
#   0x02  STOP        0x00        0x00          0x00        0x00
#   0x03  SERVO       servo_id    angle(0-180)  0x00        0x00
#   0x04  GRIP        state       0x00          0x00        0x00
#   0x05  ARM         state       0x00          0x00        0x00
#
#
#   MOTORS:
#     left_dir / right_dir  : 0x00 = forward, 0x01 = reverse
#     left_speed/right_speed: 0–255
#
#   SERVO:
#     servo_id: 0-indexed
#     angle   : 0–180 degrees
#
#   GRIP:
#     state: 0x01 = close, 0x00 = open
#
# ── Mega Reply ───────────────────────────────────────────────────────────────
#
#   'OK\n'        command executed successfully
#   'ERR\n'       checksum mismatch or unknown command
#
# ─────────────────────────────────────────────────────────────────────────────

HEADER = 0xFF

CMD_MOTORS = 0x01
CMD_STOP   = 0x02
CMD_SERVO  = 0x03
CMD_GRIP   = 0x04
CMD_ARM = 0x05

# Motor directions
MT_FWD = 0x00 # Forward
MT_RVS = 0x01 # Reverse

# Grip states
G_OPEN = 0x00
G_CLOSE = 0x01

# Arm states
A_DOWN = 0x00
A_UP = 0x01

def build_packet(cmd: int, b1: int = 0, b2: int = 0, b3: int = 0, b4: int = 0) -> bytes:
    checksum = (HEADER + cmd + b1 + b2 + b3 + b4) % 256
    return bytes([HEADER, cmd, b1, b2, b3, b4, checksum])