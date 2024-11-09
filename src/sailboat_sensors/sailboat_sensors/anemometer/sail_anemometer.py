import serial

class SailAnemometer:
    """
    This class interfaces directly with the serial port for the sensor communications.
    It is responsible for reading the wind data from the sensor.
    """
    def __init__(self, port, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = serial.Serial(self.port, baudrate=self.baudrate)
    
    def read_wind(self):
        """
        Read the wind data from the serial port.
        :return: Wind direction as an integer.
        """
        try:
            if self.serial_conn.in_waiting > 0:
                # Read the incoming message from the serial port
                wind_dir = self.serial_conn.readline().decode('utf-8').strip()
                return int(wind_dir)
        except Exception as e:
            print(f"Error reading anemometer data: {e}")
            return None

    def close(self):
        """
        Close the serial connection.
        """
        self.serial_conn.close()
