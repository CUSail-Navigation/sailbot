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
        self.wind_angle = random.randint(0, 360)
        self.wind_step = random.uniform(-5, 5) 
        self.last_sail = 0
        self.last_rudder = 0
        self.buoy_angle = 90
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
        data["buoy_angle"] = self.buoy_angle
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
    

    def send_command(self, sail, rudder, buoy_displacement):
        """
        Send a properly formatted command packet to the servo.

        :param sail: Sail position (integer)
        :param tail: Tail position (integer)
        """
        self.last_sail = sail
        self.last_rudder = rudder
        command_packet = bytearray([self.START_BYTE, sail, rudder, buoy_displacement, self.END_BYTE])

        print(f'Sent to FakeTeensy: {command_packet}')
