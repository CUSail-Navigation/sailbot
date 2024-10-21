from Adafruit_ADS1x15 import ADS1x15 as ADS
import serial
import threading

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
        """Parses a line of AirMar data and updates the readings dictionary. Some
        readings are omitted."""
        try:
            args = line.split(',')
            label = args[0][(args[0].index("$")):] 

            if "HDG" in label:
                self.readings['heading'] = args[1]
            elif "ROT" in label:
                # Wouldn't use yet--not sure about the units of measurement
                self.readings['rateOfTurn'] = args[1]  
            elif "GLL" in label:
                self.readings['latitude'] = args[1]
                self.readings['latDirection'] = args[2]
                self.readings['longitude'] = args[3]
                self.readings['longDirection'] = args[4]
            elif "VTG" in label:
                self.readings['speed'] = args[1]
        except:
            print("error parsing")

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