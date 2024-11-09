import numpy as np
import time

class FakeAirMar:
    """
    A fake data generator class for testing purposes.
    This class simulates the behavior of the real SailAirMar class,
    providing random data for GPS and IMU readings.
    """
    def __init__(self):
        # Initialize with a random starting position and heading
        # self.latitude = np.random.uniform(-80, 84)

        # self.longitude = np.random.uniform(-180, 180)
        self.latitude = 42.876379
        self.longitude = -77.007822
        self.heading = np.random.uniform(0, 360)
        self.rate_of_turn = np.random.uniform(-1, 1)  # Random rate of turn in degrees per second

    def readAirMarHeading(self):
        # Simulate heading change
        self.heading += self.rate_of_turn
        if self.heading >= 360:
            self.heading -= 360
        elif self.heading < 0:
            self.heading += 360
        return self.heading

    def readAirMarLatitude(self):
        # Simulate slight latitude changes over time
        self.latitude += np.random.uniform(-0.0001, 0.0001)
        # Ensure the latitude stays within valid range
        if self.latitude > 84:
            self.latitude = 84
        elif self.latitude < -80:
            self.latitude = -80
        return self.latitude

    def readAirMarLongitude(self):
        # Simulate slight longitude changes over time
        self.longitude += np.random.uniform(-0.0001, 0.0001)
        # Ensure the longitude stays within valid range
        if self.longitude > 180:
            self.longitude = -180 + (self.longitude - 180)
        elif self.longitude < -180:
            self.longitude = 180 - (-180 - self.longitude)
        return self.longitude

    def readAirMarROT(self):
        # Return a random rate of turn value
        return self.rate_of_turn

    def update_rate_of_turn(self):
        # Update the rate of turn to simulate changing behavior
        self.rate_of_turn = np.random.uniform(-1, 1)

    def simulate(self, duration=60):
        """
        Simulate the sensor data generation for a given duration in seconds.
        """
        start_time = time.time()
        while time.time() - start_time < duration:
            print(f"Latitude: {self.readAirMarLatitude():.6f}, Longitude: {self.readAirMarLongitude():.6f}, "
                  f"Heading: {self.readAirMarHeading():.2f}, ROT: {self.readAirMarROT():.2f}")
            time.sleep(1)
            self.update_rate_of_turn()


# Example usage
if __name__ == '__main__':
    fake_airmar = FakeAirMar()
    fake_airmar.simulate(10)
