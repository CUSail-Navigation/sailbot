import random

class TeensyFake: 
    """
    Simulates communication with a Teensy 4.0 via a serial connection. Class methods
    send fake serial messages to the Teensy, receive and process telemetry data,
    and perform any necessary conversions of transmitted and received data.
    """

    START_BYTE = 0xFF
    END_BYTE = 0xEE
    def __init__(self, ):
        self.wind_angle = random.randint(160, 200) # Tacking range.
        self.wind_step = random.uniform(-5, 5) 
        self.last_sail = 0
        self.last_rudder = 0
        self.last_jib_angle = 0
        self.last_jib_side_flag = 0
        self.dropped_packets = 0
        print("Simulated TeensyFake initialized.")

    def read_telemetry(self, data):
        """
        Reads a Teensy packet and writes the packet data.
        :param data: where the Teensy packet data is written to.
        :return: 0 for success, 1 for failure.
        """
        data["wind_angle"] = self._generate_random_wind()
        data["sail_angle"] = self.last_sail
        data["rudder_angle"] = self.last_rudder
        data["jib_angle"] = self.last_jib_angle
        data["jib_side_flag"] = self.last_jib_side_flag
        data["dropped_packets"] = self.dropped_packets
        return random.randint(0,1)

    def _generate_random_wind(self):
        """
        Generate simulated wind angle reading from the anemometer.
        :return: simulated wind direction as an integer (0 to 360 degrees).
        """
        # Update wind angle with the step value, wrapping around if necessary.
        self.wind_angle += self.wind_step
        if self.wind_angle >= 360: self.wind_angle -= 360
        elif self.wind_angle < 0: self.wind_angle += 360

        self.wind_step = random.uniform(-5, 5)
        return int(self.wind_angle)

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
            rudder = max(min(rudder, 127), -128)
        
            # Convert control values to signed 8-bit integers (bytes).
            sail_byte = sail & 0xFF if sail >= 0 else (sail + 256) & 0xFF
            rudder_byte = rudder & 0xFF if rudder >= 0 else (rudder + 256) & 0xFF
            jib_angle_byte = int(jib_angle) & 0xFF
            jib_side_byte = int(jib_side_flag) & 0xFF

            # Command payload bytes between start/end flags:
            # [sail_angle, rudder_angle, jib_angle, jib_side_flag]
            command_packet = bytearray([self.START_BYTE, sail_byte, rudder_byte, jib_angle_byte, jib_side_byte, self.END_BYTE])

            self.last_sail = int(sail)
            self.last_rudder = int(rudder)
            self.last_jib_angle = int(jib_angle)
            self.last_jib_side_flag = int(jib_side_flag)

            # Send the packet over serial.
            return 0   
        except: return 1

        print(f'Sent to FakeTeensy: {command_packet}')
