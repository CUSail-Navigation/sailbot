import random

class FakeSailAnemometer:
    """
    This class simulates the behavior of a real anemometer sensor.
    It generates random wind angle values for testing purposes.
    """

    def __init__(self):
        # Initialize the wind angle with a random value between 0 and 360 degrees
        self.wind_angle = random.randint(0, 360)
        # Set a step value to simulate changes in wind angle
        self.step = random.uniform(-5, 5)  # Simulate changes in wind angle between -5 and +5 degrees

    def read_wind(self):
        """
        Simulate reading the wind angle from the sensor.
        :return: Wind direction as an integer (0 to 360 degrees).
        """
        # Update wind angle with the step value, wrapping around if necessary
        self.wind_angle += self.step
        if self.wind_angle >= 360:
            self.wind_angle -= 360
        elif self.wind_angle < 0:
            self.wind_angle += 360

        # Randomly change the step value to simulate unpredictable wind changes
        self.step = random.uniform(-5, 5)

        # Return the wind angle as an integer
        return int(self.wind_angle)

    def close(self):
        """
        Simulate closing the connection to the anemometer (no-op for fake sensor).
        """
        pass
