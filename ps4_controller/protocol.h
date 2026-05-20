#ifndef S_PROTOCOL
#define S_PROTOCOL

// ─────────────────────────────────────────────────────────
// Packet Format
//
// [HEADER] [CMD] [B1] [B2] [B3] [B4] [CHECKSUM]
//
// Total: 7 bytes
// ─────────────────────────────────────────────────────────

#define PACKET_SIZE 7
#define HEADER 0xFF

// ── Commands ─────────────────────────────────────────────

#define CMD_MOTORS 0x01
#define CMD_STOP   0x02
#define CMD_SERVO  0x03
#define CMD_GRIP   0x04
#define CMD_ARM    0x05

// ── Motor Directions ────────────────────────────────────

#define MT_FWD 0x00
#define MT_RVS 0x01

// ── Grip States ─────────────────────────────────────────

#define G_OPEN  0x00
#define G_CLOSE 0x01

// ── Arm States ──────────────────────────────────────────

#define A_DOWN 0x00
#define A_UP   0x01

// ── Packet Struct ───────────────────────────────────────

struct Packet {
    uint8_t header;
    uint8_t cmd;
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;
    uint8_t b4;
    uint8_t checksum;
};

// ── Checksum ────────────────────────────────────────────

inline uint8_t computeChecksum(
    uint8_t header,
    uint8_t cmd,
    uint8_t b1,
    uint8_t b2,
    uint8_t b3,
    uint8_t b4
);

// ── Validate Packet ─────────────────────────────────────

inline bool validatePacket(const Packet &p);



inline bool readPacket(HardwareSerial &serial, Packet &p);

#endif