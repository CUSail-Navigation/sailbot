import serial

class TeensyHardware:
    START_BYTE = 0xFF
    END_BYTE = 0xFE

    """
    Handles real servo communication via serial connection.
    Sends commands to a physical servo device.
    """
    def __init__(self, port):
        self.port = port
        self.buffer = []
        try:
            self.serial = serial.Serial(self.port, baudrate=9600)
            print(f"Serial connection established on {self.port}.")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {self.port}: {e}")
            raise

    def is_telemetry(self):
        return self.serial.in_waiting > 0
        
    def read_telemetry(self):
        """
        Receives teensy packet and returns the packet data
        :param packet: Teensy packet
        """
        if self.serial.in_waiting > 0:
            incoming_byte = self.serial.read()

            # reset buffer if we have a new packet
            if incoming_byte == self.START_BYTE:
                self.buffer = []

            # add data to the packet
            self.buffer.append(incoming_byte)

            # if we have a full packet, return the data
            if incoming_byte == self.END_BYTE and len(self.buffer) == 4:
                return self.parse_packet(self.buffer)


    def _parse_packet(packet):
        """
        Parse a packet from the Teensy.
        :param packet: Packet from the Teensy
        """
        wind_angle = packet[1]
        sail_angle = packet[2]
        rudder_angle = packet[3]
        dropped_packets = packet[4]
        return wind_angle, sail_angle, rudder_angle, dropped_packets


    def send_command(self, sail, tail):
        """
        Send a properly formatted command packet to the servo.

        :param sail: Sail position (integer)
        :param tail: Tail position (integer)
        """
        try:
            # Create the packet: [start flag] [sail] [tail] [end flag]
            command_packet = bytearray([START_BYTE, sail, tail, END_BYTE])

            # Send the packet over serial
            self.serial.write(command_packet)
            print(f'Sent to servo: {command_packet}')
        except Exception as e:
            print(f"Failed to write to serial port: {e}")


    def close(self):
        """
        Close the serial connection.
        """
        if hasattr(self, 'serial'):
            self.serial.close()
            print('Serial connection closed.')
