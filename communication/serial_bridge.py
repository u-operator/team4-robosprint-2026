import serial
import threading

class SerialBridge:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._lock = threading.Lock()

    def send(self, packet: bytes) -> str:
        """Send packet, wait for ACK. Returns 'OK', 'ERR...' or '' on timeout."""
        with self._lock:
            self.ser.write(packet)
            return self.ser.readline().decode().strip()

    def send_no_wait(self, packet: bytes):
        """Fire and forget — for high-frequency updates."""
        with self._lock:
            self.ser.write(packet)

    def close(self):
        self.ser.close()