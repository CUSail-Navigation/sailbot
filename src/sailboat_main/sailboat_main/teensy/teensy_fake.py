import random

class TeensyFake: 
    """
    Simulates communication with a Teensy 4.0 via a serial connection. Class methods
    send fake serial messages to the Teensy, receive and process telemetry data,
    and perform any necessary conversions of transmitted and received data.
    """

    # CONSTANTS.
    START_BYTE = 0xFF
    END_BYTE = 0xEE

    def __init__(self):
        self.wind_angle = random.randint(160, 200) # Tacking range.
        self.wind_step = random.uniform(-5, 5) 
        self.last_mainsail_angle = 0
        self.last_rudder_angle = 0
        self.last_jib_angle = 0
        self.last_jib_side_flag = 0
        self.dropped_packets = 0
        print("Simulated TeensyFake initialized.")

    def read_telemetry(self, data):
        """
        Reads a Teensy telemetry packet and records the packet data.
        The format of the telemetry payload between start/end flags is:
            [wind_hi, wind_lo, mainsail_angle, rudder_angle, jib_angle, jib_side_flag, dropped_packets]
        :param data: where the Teensy packet is written to.
        :return: 0 for success, 1 for failure.
        """
        data["wind_angle"] = self._generate_random_wind()
        data["mainsail_angle"] = self.last_mainsail_angle
        data["rudder_angle"] = self.last_rudder_angle
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

            self.last_mainsail_angle = int(mainsail_angle)
            self.last_rudder_angle = int(rudder_angle)
            self.last_jib_angle = int(jib_angle)
            self.last_jib_side_flag = int(jib_side_flag)

            # Send the packet over serial.
            return 0   
        except: return 1

        print(f'Sent to FakeTeensy: {command_packet}')
