import serial

class TeensyHardware:
    """
    Handles communication with a Teensy 4.0 via a serial connection. Class methods
    send serial messages to the Teensy, receives and processes telemetry data, 
    and performs any necessary conversions of transmitted and received data.
    """

    START_BYTE = 0xff
    END_BYTE = 0xee
    PACKET_LENGTH = 5

    def __init__(self, port):
        self.port = port
        self.buffer = []
        self.packet_started = False
        try:
            self.serial = serial.Serial(self.port, baudrate=9600)
        except serial.SerialException as e:
            raise
        
    def read_telemetry(self, data):
        """
        Receives teensy packet and returns the packet data
        :param packet: Teensy packet
        """
        # check for waiting serial data
        while self.serial.in_waiting > 0:
            incoming_byte = self.serial.read()

            # if we see a packet start byte, set flags, clear buffer
            if incoming_byte == self.START_BYTE.to_bytes(1, 'big'):
                self.packet_started = True
                self.buffer = []
            # if we see a packet end byte, process the buffer data
            elif incoming_byte == self.END_BYTE.to_bytes(1, 'big') and len(self.buffer) == 4:
                self.packet_started = False
                data["wind_angle"], \
                data["sail_angle"], \
                data["rudder_angle"], \
                data["dropped_packets"] = self._parse_packet(self.buffer)

                # clear buffer
                self.serial.reset_input_buffer()
                return 0
            # if we have previously seen a packet start byte, add data to our buffer
            elif self.packet_started:
                self.buffer.append(int.from_bytes(incoming_byte, 'big'))
        else:
            return 1
    


    def _parse_packet(self, packet):
        """
        Parse a packet from the Teensy.
        :param packet: Packet from the Teensy
        """
        wind_angle = (packet[0] << 8) | packet[1]

        # uint8_t to int8_t conversion
        if packet[2] >= 128:
            sail_angle = packet[2] - 256
        else:
            sail_angle = packet[2]
        # uint8_t to int8_t conversion
        if packet[3] >= 128:
            rudder_angle = packet[3] - 256
        else:
            rudder_angle = packet[3]

        dropped_packets = packet[4]

        return wind_angle, sail_angle, rudder_angle, dropped_packets


    def send_command(self, sail, rudder, mode):
        """
        Send a properly formatted command packet to the servo.

        :param sail: Sail position (integer)
        :param tail: Tail position (integer)
        :param mode: Control mode (boolean)
        """
        try:
            # check bounds
            sail = max(min(sail, 127), -128)
            rudder = max(min(rudder, 127), -128)
        
            # convert sail and tail to signed 8-bit integers (bytes)
            sail_byte = sail & 0xFF if sail >= 0 else (sail + 256) & 0xFF
            rudder_byte = rudder & 0xFF if rudder >= 0 else (rudder + 256) & 0xFF

            # convert mode flag to byte
            mode_byte = 1 if mode else 0

            # create the packet: [start flag] [mode] [sail] [rudder] [end flag]
            command_packet = bytearray([self.START_BYTE, mode_byte, sail_byte, rudder_byte, self.END_BYTE])

            # send the packet over serial
            self.serial.write(command_packet)
            return 0   
        except:
            return 1

    def close(self):
        """
        Close the serial connection.
        """
        if hasattr(self, 'serial'):
            self.serial.close()
            print('Serial connection closed.')
