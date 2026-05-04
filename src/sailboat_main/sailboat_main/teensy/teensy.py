import serial

class TeensyHardware:
    """
    Handles communication with a Teensy 4.0 via a serial connection. Class methods
    send serial messages to the Teensy, receive and process telemetry data,
    and perform any necessary conversions of transmitted and received data.
    """

    START_BYTE = 0xFF
    END_BYTE = 0xEE
    BAUD_RATE = 9600
    # Telemetry payload bytes between start/end flags:
    # [wind_hi, wind_lo, mainsail_angle, rudder_angle, jib_angle, jib_side_flag, dropped_packets]
    PACKET_LENGTH = 7

    def __init__(self, port):
        self.port = port
        self.buffer = []
        self.packet_started = False
        try:
            self.serial = serial.Serial(self.port, baudrate=self.BAUD_RATE)
        except serial.SerialException as e:
            raise
        
    def read_telemetry(self, data):
        """
        Reads a Teensy packet and records the packet data.
        :param data: where the Teensy packet data is written to.
        :return: 0 for success, 1 for failure.
        """
        # Check for waiting serial data.
        while self.serial.in_waiting > 0:
            incoming_byte = self.serial.read()

            # If we see a packet start byte: set flags, clear buffer.
            if incoming_byte == self.START_BYTE.to_bytes(1, 'big'):
                self.packet_started = True
                self.buffer = []
            # If we see a packet end byte, process the buffer data:
            elif incoming_byte == self.END_BYTE.to_bytes(1, 'big') and len(self.buffer) == self.PACKET_LENGTH:
                self.packet_started = False
                data["wind_angle"], \
                data["sail_angle"], \
                data["rudder_angle"], \
                data["jib_angle"], \
                data["jib_side_flag"], \
                data["dropped_packets"] = self._parse_packet(self.buffer)
                self.serial.reset_input_buffer() # Clear buffer.
                return 0
            # If we have previously seen a packet start byte, add data to our buffer.
            elif self.packet_started:
                self.buffer.append(int.from_bytes(incoming_byte, 'big'))
        else:
            return 1

    def _parse_packet(self, packet):
        """
        Parse a packet from the Teensy.
        :param packet: the packet from the Teensy.
        """
        wind_angle = (packet[0] << 8) | packet[1]
        # Keep existing sail/rudder conversion behavior unchanged.
        # uint8_t to int8_t conversion for negative values.
        if packet[2] >= 128: sail_angle = packet[2] - 256
        else: sail_angle = packet[2]

        if packet[3] >= 128: rudder_angle = packet[3] - 256
        else: rudder_angle = packet[3]

        if packet[4] >= 128: jib_angle = packet[4] - 256
        else: jib_angle = packet[4]

        jib_side_flag = packet[5]
        dropped_packets = packet[6]

        return wind_angle, sail_angle, rudder_angle, jib_angle, jib_side_flag, dropped_packets


    def send_command(self, sail, rudder, jib_angle=0, jib_side_flag=0):
        """
        Send a properly formatted command packet to the servo.
        :param sail: mainsail position (integer).
        :param rudder: rudder position (integer).
        :param jib_angle: jib position (integer).
        :param jib_side_flag: side for the jib (integer).
        """
        try:
            # Check bounds.
            sail = max(min(sail, 127), -128)
            rudder = rudder + 25
        
            # Convert control values to signed 8-bit integers (bytes).
            sail_byte = sail & 0xFF if sail >= 0 else (sail + 256) & 0xFF
            rudder_byte = rudder & 0xFF if rudder >= 0 else (rudder + 256) & 0xFF
            jib_angle_byte = int(jib_angle) & 0xFF
            jib_side_byte = int(jib_side_flag) & 0xFF

            # Command payload bytes between start/end flags:
            # [sail_angle, rudder_angle, jib_angle, jib_side_flag]
            command_packet = bytearray([self.START_BYTE, sail_byte, rudder_byte, jib_angle_byte, jib_side_byte, self.END_BYTE])

            # Send the packet over serial.
            self.serial.write(command_packet)
            return 0   
        except: return 1

    def close(self):
        """
        Close the serial connection.
        """
        if hasattr(self, 'serial'):
            self.serial.close()
            print('Serial connection closed.')
