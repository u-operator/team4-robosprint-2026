#include "protocol.h"
// ── Checksum ────────────────────────────────────────────

inline uint8_t computeChecksum(
    uint8_t header,
    uint8_t cmd,
    uint8_t b1,
    uint8_t b2,
    uint8_t b3,
    uint8_t b4
) {
    return (header + cmd + b1 + b2 + b3 + b4) & 0xFF;
}

inline bool validatePacket(const Packet &p) {

    if (p.header != HEADER)
        return false;

    uint8_t calc =
        computeChecksum(
            p.header,
            p.cmd,
            p.b1,
            p.b2,
            p.b3,
            p.b4
        );

    return (calc == p.checksum);
}

// ─────────────────────────────────────────────────────────
// Robust Packet Reader
//
// Features:
// - Resynchronizes automatically
// - Waits for HEADER byte
// - Handles corrupted/misaligned packets
// - Non-blocking
// ─────────────────────────────────────────────────────────

inline bool readPacket(HardwareSerial &serial, Packet &p) {

    static uint8_t buffer[PACKET_SIZE];
    static uint8_t index = 0;

    while (serial.available()) {

        uint8_t byteIn = serial.read();

        // Wait for header
        if (index == 0) {

            if (byteIn != HEADER)
                continue;

            buffer[index++] = byteIn;
        }
        else {

            buffer[index++] = byteIn;

            // Full packet received
            if (index >= PACKET_SIZE) {

                // Copy into struct
                p.header   = buffer[0];
                p.cmd      = buffer[1];
                p.b1       = buffer[2];
                p.b2       = buffer[3];
                p.b3       = buffer[4];
                p.b4       = buffer[5];
                p.checksum = buffer[6];

                // Reset for next packet
                index = 0;

                // Validate
                if (validatePacket(p)) {
                    return true;
                }

                // Invalid packet
                return false;
            }
        }
    }

    return false;
}