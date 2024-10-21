import serial

class SailServo:
    """
    Handles real servo communication via serial connection.
    Sends commands to a physical servo device.
    """
    def __init__(self, port):
        self.port = port
        try:
            self.servo_serial = serial.Serial(self.port, baudrate=9600)
            print(f"Serial connection established on {self.port}.")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {self.port}: {e}")
            raise

    def send_command(self, sail, tail):
        """
        Send a command to the servo.
        :param sail: Sail position
        :param tail: Tail position
        """
        try:
            command = f"{sail} {tail}\n".encode('utf-8')
            self.servo_serial.write(command)
            print(f'Sent to servo: {command}')
        except Exception as e:
            print(f"Failed to write to serial port: {e}")

    def close(self):
        """
        Close the serial connection.
        """
        if hasattr(self, 'servo_serial'):
            self.servo_serial.close()
            print('Serial connection closed.')
