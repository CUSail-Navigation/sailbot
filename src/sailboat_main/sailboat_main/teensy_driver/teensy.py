import serial

class TeensyHardware:

    # Buffer to hold incoming bytes
    buffer = []
    START_BYTE = 0xFF
    END_BYTE = 0xFE

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

    def run_step(self, sail, tail):
        """
        Receives teensy packet and responds.
        :param packet: Teensy packet
        """
        # Read incoming bytes
        if self.servo_serial.in_waiting > 0:
            incoming_byte = self.servo_serial.read()
            # Check if the byte is the start byte
            if incoming_byte == self.START_BYTE:
                # Clear the buffer
                self.buffer = []
            # Add the byte to the buffer
            self.buffer.append(incoming_byte)
            # Check if the byte is the end byte
            if incoming_byte == self.END_BYTE:
                # Parse the packet
                wind_angle, sail_angle, rudder_angle, dropped_packets = self.parse_packet(self.buffer)
                # Send the command
                self.send_command(sail, tail)


    def parse_packet(packet):
        """
        Parse a packet from the Teensy.
        :param packet: Packet from the Teensy
        """
        # Convert packet bytes to integers, assuming each value is a byte
        wind_angle = packet[1]
        sail_angle = packet[2]
        rudder_angle = packet[3]
        dropped_packets = packet[4]
        return wind_angle, sail_angle, rudder_angle, dropped_packets


    def send_command(self, sail, tail):
        """
        Send a command to the servo.
        :param sail: Sail position
        :param tail: Tail position
        """
        try:
            command = f"FF {sail} {tail} FE\n".encode('utf-8')
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
