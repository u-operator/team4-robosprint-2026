#ifndef S_PROTOCOL
#define S_PROTOCOL

#include <Arduino.h>
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
#define CMD_EDOWN  0xD1
#define CMD_EUP    0xD0
#define CMD_GROT   0x06
#define CMD_RELAY  0x07

// ── Motor Directions ────────────────────────────────────

#define MT_FWD 0x00
#define MT_RVS 0x01

// ── Grip States ─────────────────────────────────────────

#define G_OPEN  0x00
#define G_CLOSE 0x01

// ── Arm States ──────────────────────────────────────────

#define A_CW   0x01
#define A_CCW  0x02
#define A_STOP 0x00

// -- Grip Rotation Direction

#define G_CW   0x01
#define G_CCW  0x02
#define G_STOP 0x00

// Relay state
#define RELAY_ON 0x01
#define RELAY_OFF 0x00


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

bool validatePacket(const Packet &p);



bool readPacket(HardwareSerial &serial, Packet &p);

#endif