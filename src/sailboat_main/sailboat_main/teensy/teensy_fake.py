import random

class TeensyFake: 
    """
    Simulates communication with a Teensy 4.0 via a serial connection. Class methods
    send fake serial messages to the Teensy, receives and processes telemetry data, 
    and performs any necessary conversions of transmitted and received data.
    """

    START_BYTE = 0xff
    END_BYTE = 0xee
    def __init__(self, ):
        self.wind_angle = random.randint(160, 200) #tacking range
        self.wind_step = random.uniform(-5, 5) 
        self.last_sail = 0
        self.last_rudder = 0
        self.dropped_packets = 0

        print("Simulated TeensyFake initialized")

    def read_telemetry(self, data):
        """
        Return simulated packet data
        :param packet: Teensy packet
        """
        data["wind_angle"] = self._generate_random_wind()
        data["sail_angle"] = self.last_sail
        data["rudder_angle"] = self.last_rudder
        data["dropped_packets"] = self.dropped_packets

        return random.randint(0,1)

    def _generate_random_wind(self):
        """
        Generate simulated reading the wind angle from the sensor.
        :return: Wind direction as an integer (0 to 360 degrees).
        """
        # Update wind angle with the step value, wrapping around if necessary
        self.wind_angle += self.wind_step
        if self.wind_angle >= 360:
            self.wind_angle -= 360
        elif self.wind_angle < 0:
            self.wind_angle += 360

        self.wind_step = random.uniform(-5, 5)
        return int(self.wind_angle)
    

    def send_command(self, sail, rudder):
        """
        Send a properly formatted command packet to the servo.

        :param sail: Sail position (integer)
        :param tail: Tail position (integer)
        """
        try:
            # check bounds
            sail = max(min(sail, 127), -128)
            rudder = max(min(rudder, 127), -128)
        
            # convert sail and tail to signed 8-bit integers (bytes)
            sail_byte = sail & 0xFF if sail >= 0 else (sail + 256) & 0xFF
            rudder_byte = rudder & 0xFF if rudder >= 0 else (rudder + 256) & 0xFF

            # create the packet: [start flag] [sail] [tail] [end flag]
            command_packet = bytearray([self.START_BYTE, sail_byte, rudder_byte, self.END_BYTE])

            # send the packet over serial
            return 0   
        except:
            return 1

        print(f'Sent to FakeTeensy: {command_packet}')
