import numpy as np
import time

class FakeVectorNav:
    """
    A fake data generator class for testing purposes.
    This class simulates the behavior of the real VectorNav class,
    providing random data for GPS and IMU readings.
    """
    def __init__(self):
        # Initialize with a starting position and heading

        self.latitude = 42.876379
        self.longitude = -77.007822
        # self.heading = np.random.uniform(0, 360)
        # self.pitch = np.random.uniform(-90, 90)
        # self.roll = np.random.uniform(0, 360)
        self.heading = 0
        self.pitch = 0
        self.roll = 0
        
    def readVectorNavYaw(self):
        # Simulate heading change
        if self.heading >= 360:
            self.heading -= 360
        elif self.heading < 0:
            self.heading += 360
        return float(self.heading)

    def readVectorNavPitch(self):
        # Simulate heading change
        if self.heading >= 360:
            self.heading -= 360
        elif self.heading < 0:
            self.heading += 360
        return float(self.heading)
    
    def readVectorNavRoll(self):
        # Simulate heading change
        if self.heading >= 360:
            self.heading -= 360
        elif self.heading < 0:
            self.heading += 360
        return float(self.heading)
    
    def readVectorNavLatitude(self):
        # Simulate slight latitude changes over time
        self.latitude += np.random.uniform(-0.0001, 0.0001)
        # Ensure the latitude stays within valid range
        if self.latitude > 84:
            self.latitude = 84
        elif self.latitude < -80:
            self.latitude = -80
        return self.latitude

    def readVectorNavLongitude(self):
        # Simulate slight longitude changes over time
        self.longitude += np.random.uniform(-0.0001, 0.0001)
        # Ensure the longitude stays within valid range
        if self.longitude > 180:
            self.longitude = -180 + (self.longitude - 180)
        elif self.longitude < -180:
            self.longitude = 180 - (-180 - self.longitude)
        return self.longitude

    def simulate(self, duration=60):
        """
        Simulate the sensor data generation for a given duration in seconds.
        """
        start_time = time.time()
        while time.time() - start_time < duration:
            print(f"Latitude: {self.readVectorNavLatitude():.6f}, Longitude: {self.readVectorNavLongitude():.6f}, "
                  f"Yaw: {self.readVectorNavYaw():.2f}, Pitch: {self.readVectorNavPitch()}, Roll: {self.readVectorNavRoll()}" )
            time.sleep(1)


# Example usage
if __name__ == '__main__':
    fake_vectornav = FakeVectorNav()
    fake_vectornav.simulate(10)

