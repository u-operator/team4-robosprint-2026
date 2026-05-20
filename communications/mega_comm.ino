#define HEADER      0xFF
#define CMD_MOTORS  0x01
#define CMD_STOP    0x02
#define CMD_SERVO   0x03
#define CMD_GRIP    0x04

#define PACKET_SIZE 5

byte buf[PACKET_SIZE];

void loop() {
    if (Serial.available() >= PACKET_SIZE) {
        // Wait for header
        if (Serial.peek() != HEADER) { Serial.read(); return; }

        Serial.readBytes(buf, PACKET_SIZE);

        // Validate checksum
        byte expected = (buf[0] + buf[1] + buf[2] + buf[3]) % 256;
        if (buf[4] != expected) {
            Serial.println("ERR checksum");
            return;
        }

        byte cmd = buf[1];
        byte b1  = buf[2];
        byte b2  = buf[3];

        switch (cmd) {
            case CMD_MOTORS: {
                int left  = (int)b1 - 128;   // undo offset
                int right = (int)b2 - 128;
                setMotors(left, right);
                Serial.println("OK");
                break;
            }
            case CMD_STOP:
                setMotors(0, 0);
                Serial.println("OK");
                break;
            case CMD_SERVO:
                servos[b1].write(b2);   // b1=id, b2=angle
                Serial.println("OK");
                break;
            case CMD_GRIP:
                // b1: 1=close, 0=open
                digitalWrite(GRIP_PIN, b1 ? HIGH : LOW);
                Serial.println("OK");
                break;
        }
    }
}