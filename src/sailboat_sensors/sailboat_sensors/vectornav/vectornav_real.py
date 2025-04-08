import serial
import threading

class SailVectorNav:
    """
    SailVectorNav implements a class used to connect the component to its proper
    communication protocol. This class also implements functions to return raw 
    data from the VectorNav.
    """
    
    def __init__(self, port):
        """Initializes the VectorNav. Creates a seperate thread to continuously 
        read from the serial output."""
        self.ser = serial.Serial(port, 115200)
        self.readings = {"yaw":0, "pitch":0, "roll":0, "latitude":0,"longitude":0}
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
                    self.parseVectorNavData(line)

    def parseVectorNavData(self, line):
        """"Parses a line of VectorNav data and updates the readings dictionary."""
        try:
            args = line.split(',')
            self.readings['yaw'] = args[4]
            self.readings['pitch'] = args[5]
            self.readings['roll'] = args[6]
            self.readings['latitude'] = args[7]
            self.readings['longitude'] = args[8]
        except:
            print("error parsing")
    
    def readVectorNavYaw(self):
        """Returns the yaw in degrees. 0 degrees is North, 90 degrees is East,
        180 degrees is South, 270 degrees is West.""" # TODO: Validate this comment
        with self.lock:
            return (450.0 - float(self.readings['yaw'])) % 360.0
    
    def readVectorNavPitch(self):
        """Returns the pitch in degrees. 0 degrees is level, positive is bow up,
        negative is bow down."""
        with self.lock:
            return float(self.readings['pitch'])

    def readVectorNavRoll(self):
        """Returns the roll in degrees. 0 degrees is level, positive is starboard
        up, negative is port up."""
        with self.lock:
            return float(self.readings['roll'])
    
    def readVectorNavLatitude(self):
        """Returns the latitude in degrees"""
        with self.lock:
            return float(self.readings['latitude'])
    
    def readVectorNavLongitude(self):
        """Returns the longitude in degrees"""
        with self.lock:
            return float(self.readings['longitude'])

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