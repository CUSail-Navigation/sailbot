from Adafruit_ADS1x15 import ADS1x15 as ADS
from math import radians, cos, sin, sqrt, atan2
import serial
import threading
import time

IMU_ADDRESS = 0x77
ADC_ADDRESS = 0x48


class SailAirMar:
    """
    SailAirMar implements a class used to connect the component to its proper
    communication protocol. This class also implements functions to return raw 
    data from the AirMar.
    """
    
    def __init__(self, port):
        """Initializes the AirMar. Creates a seperate thread to continuously 
        read from the serial output."""
        self.ser = serial.Serial(port, 9600)
        self.readings = {"heading":0, "rateOfTurn":0, "latitude":0, "longitude":0,"latDirection":"S", "longDirection":"W"}
        self.lock = threading.Lock()
        self.gps_history = [] # Stores the last two GPS data points (lat, long, timestamp)
        reader_thread = threading.Thread(target=self.serialDataReader)
        reader_thread.daemon = True 
        reader_thread.start()
    
    def serialDataReader(self):
        """Reads data from the serial port and stores it in a dictionary."""
        while True:
            line = self.ser.readline().decode().strip()
            if line:
                with self.lock:
                    self.parseAirMarData(line)

    def parseAirMarData(self, line):
        """Parses a line of AirMar data and updates the readings dictionary."""
        try:
            args = line.split(',')
            label = args[0][(args[0].index("$")):]

            if "HDG" in label:
                self.readings['heading'] = args[1]
            elif "ROT" in label:
                self.readings['rateOfTurn'] = args[1]
            elif "GLL" in label:
                self.readings['latitude'] = args[1]
                self.readings['latDirection'] = args[2]
                self.readings['longitude'] = args[3]
                self.readings['longDirection'] = args[4]
                lat = self.readAirMarLatitude()
                lon = self.readAirMarLongitude()
                self.gps_history.append((lat, lon, time.time()))
                if len(self.gps_history) > 2:
                    self.gps_history.pop(0)
            elif "VTG" in label:
                self.readings['speed'] = args[1]
        except Exception as e:
            print(f"Error parsing: {e}")

    def readAirMarHeading(self):
        """Returns the heading in degrees. 0 degrees is East, 90 degrees is North,
        180 degrees is West, 270 degrees is South."""
        return float(self.readings['heading'])
            # ~ if (self.readings['heading'] != ''):
                # ~ return self._convertToPolar(self.readings['heading'])
            # ~ else:
                # ~ return 0
        
    def readAirMarROT(self):
        """Returns the rate of turn in degrees per second."""
        with self.lock:
            return self._convertDegreePerSec(self.readings['rateOfTurn'])
        
    def readAirMarLatitude(self):
        """Returns the latitude in degrees"""
        with self.lock:
            if self.readings['latitude'] != '':
                lat = self._defConvertMins(self.readings['latitude'])
                if self.readings['latDirection'] == 'S':
                    return -lat
                return lat
            else:
                return 0

    def readAirMarLongitude(self):
        """Retruns the longitude in degrees"""
        with self.lock:
            if self.readings['longitude'] != '':
                long = self._defConvertMins(self.readings['longitude'])
                if self.readings['longDirection'] == 'W':
                    return -long
                return long
            else:
                return 0
    
    def readAirMarVelocity(self):
        """Calculates and returns the velocity based on the last two GPS data points."""
        with self.lock:
            if len(self.gps_history) < 2:
                return 0  # Not enough data to calculate velocity
            
            # Extract the last two GPS data points
            lat1, lon1, t1 = self.gps_history[-2]
            lat2, lon2, t2 = self.gps_history[-1]
            
            # Calculate the distance between the two points
            distance = self._haversine(lat1, lon1, lat2, lon2)
            
            # Calculate the time difference
            time_diff = t2 - t1
            if time_diff <= 0:
                return 0
            
            # Calculate velocity
            return  abs(distance / time_diff)

    def _haversine(self, lat1, lon1, lat2, lon2):
        """Calculates the great-circle distance between two points in meters."""
        R = 6371000  # Earth radius in meters
        phi1, phi2 = radians(lat1), radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)

        a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c

    def _convertToPolar(self,raw_heading):
        """Converts the heading to polar coordinates."""
        res = float(raw_heading) - 90
        if res < 0:
            return res + 360
        return res

    def _convertDegreePerSec(self,raw_rot):
        """Converts the heading to degrees per second."""
        return float(raw_rot)/60.0

    def _defConvertMins(self,minutes):
        """Extract the degrees based on the format (DDMM.mmmm)"""
        degrees = float(minutes) // 100
        minutes_only = float(minutes) % 100
        return degrees + (minutes_only / 60)