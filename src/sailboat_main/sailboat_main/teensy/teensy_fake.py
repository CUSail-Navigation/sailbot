import random

class TeensyFake: 
    START_BYTE = 0xFF
    END_BYTE = 0xFE

    """
    Handles real servo communication via serial connection.
    Sends commands to a physical servo device.
    """
    def __init__(self, ):
        self.wind_angle = random.randint(0, 360)
        self.wind_step = random.uniform(-5, 5) 
        self.last_sail = 0
        self.last_rudder = 0
        self.dropped_packets = 0

        print("Simulated TeensyFake initialized")

    def is_telemetry(self):
        return random.randint(0,1)

    def read_telemetry(self):
        """
        Return simulated packet data
        :param packet: Teensy packet
        """
        return self._generate_random_wind(), self.last_sail, self.last_tail, self.dropped_packets

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
    

    def send_command(self, sail, tail):
        """
        Send a properly formatted command packet to the servo.

        :param sail: Sail position (integer)
        :param tail: Tail position (integer)
        """
        self.last_sail = sail
        self.last_tail = tail
        command_packet = bytearray([self.START_BYTE, sail, tail, self.END_BYTE])

        print(f'Sent to FakeTeensy: {command_packet}')
