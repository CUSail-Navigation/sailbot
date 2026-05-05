import serial

class TeensyHardware:
    """
    Handles communication with a Teensy 4.0 via a serial connection. Class methods
    send serial messages to the Teensy, receive and process telemetry data,
    and perform any necessary conversions of transmitted and received data.
    """

    # CONSTANTS.
    START_BYTE = 0xFF
    END_BYTE = 0xEE
    BAUD_RATE = 9600
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
        Reads a Teensy telemetry packet and records the packet data.
        The format of the telemetry payload between start/end flags is:
            [wind_hi, wind_lo, mainsail_angle, rudder_angle, jib_angle, jib_side_flag, dropped_packets]
        :param data: where the Teensy packet is written to.
        :return: 0 for success, 1 for failure.
        """
        # Check for waiting serial data.
        while self.serial.in_waiting > 0:
            incoming_byte = self.serial.read()

            # If we see a packet start byte: set flags, clear buffer.
            if incoming_byte == self.START_BYTE.to_bytes(1, 'big'):
                self.packet_started = True
                self.buffer = []
            # If we see a packet end byte and the buffer is full, process the buffer and store into ``data``.
            elif incoming_byte == self.END_BYTE.to_bytes(1, 'big') and len(self.buffer) == self.PACKET_LENGTH:
                self.packet_started = False
                data["wind_angle"], \
                data["mainsail_angle"], \
                data["rudder_angle"], \
                data["jib_angle"], \
                data["jib_side_flag"], \
                data["dropped_packets"] = self._parse_packet(self.buffer)
                self.serial.reset_input_buffer() # Clear buffer.
                return 0
            # If we have previously seen a packet start byte, add data to our buffer.
            elif self.packet_started:
                self.buffer.append(int.from_bytes(incoming_byte, 'big'))
        else: return 1

    def _parse_packet(self, packet):
        """
        A helper method to actually parse the data in a packet from the Teensy.
        :param packet: the packet from the Teensy.
        """
        wind_angle = (packet[0] << 8) | packet[1]

        # uint8_t to int8_t conversion for negative values.
        mainsail_angle = packet[2] - 256 if packet[2] >= 128 else packet[2]
        rudder_angle = packet[3] - 256 if packet[3] >= 128 else packet[3]
        jib_angle = packet[4] - 256 if packet[4] >= 128 else packet[4]

        jib_side_flag = packet[5]
        dropped_packets = packet[6]

        return wind_angle, mainsail_angle, rudder_angle, jib_angle, jib_side_flag, dropped_packets

    def send_command(self, mainsail_angle, rudder_angle, jib_angle, jib_side_flag):
        """
        Send a properly formatted command packet to the servo.
        :param mainsail_angle: new mainsail angle to set (integer). Should be in range [0, 90].
        :param rudder_angle: new rudder angle to set (integer). Should be in range [-45, 45].
        :param jib_angle: new jib angle to set (integer). Should be in range [10, 80].
        :param jib_side_flag: side to set the jib on (integer).
        """
        try:
            # Check bounds. For the sails, this is just defensive.
            mainsail_angle = max(min(mainsail_angle, 127), -128)
            rudder_angle = rudder_angle + 45    # (2025-2026) Rudder angle ranges from -45 to 45 degrees.
            jib_angle = max(min(jib_angle, 127), -128)

            # Convert control values to 8-bit integers (bytes). The else check is just defensive.
            mainsail_byte = mainsail_angle & 0xFF if mainsail_angle >= 0 else (mainsail_angle + 256) & 0xFF
            rudder_byte = rudder_angle & 0xFF if rudder_angle >= 0 else (rudder_angle + 256) & 0xFF
            jib_angle_byte = jib_angle & 0xFF if jib_angle >= 0 else (jib_angle + 256) & 0xFF
            jib_side_byte = jib_side_flag & 0xFF

            # Command payload format between start/end flags:
            # [mainsail_angle, rudder_angle, jib_angle, jib_side_flag]
            command_packet = bytearray([self.START_BYTE, mainsail_byte, rudder_byte, jib_angle_byte, jib_side_byte, self.END_BYTE])

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
