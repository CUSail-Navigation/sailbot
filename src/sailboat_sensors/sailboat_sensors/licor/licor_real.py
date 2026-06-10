import serial
import threading


class LicorAnemometer:
    def __init__(self, port, baud_rate=9600):
        self.readings = {'speed': 0.0, 'direction': 0}
        self.lock = threading.Lock()
        self.ser = serial.Serial(port, baud_rate)
        reader_thread = threading.Thread(target=self._read_serial)
        reader_thread.daemon = True
        reader_thread.start()

    def _read_serial(self):
        while True:
            line = self.ser.readline().decode().strip()
            if line:
                self._parse(line)

    def _parse(self, line):
        # Format: S 00.16 D 305 U 00.13 V -00.09 ...
        tokens = line.split()
        try:
            s_idx = tokens.index('S')
            d_idx = tokens.index('D')
            speed = float(tokens[s_idx + 1])
            direction = int(tokens[d_idx + 1])
            with self.lock:
                self.readings['speed'] = speed
                self.readings['direction'] = direction
        except (ValueError, IndexError):
            pass

    def read_speed(self):
        with self.lock:
            return self.readings['speed']

    def read_direction(self):
        with self.lock:
            return self.readings['direction']
