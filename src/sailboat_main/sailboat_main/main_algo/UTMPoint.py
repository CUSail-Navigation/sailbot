import LatLongPoint
import utm 

class UTMPoint():
    """
    A class to represent a point in UTM coordinates.
    """
    easting : float
    northing : float
    zone_number : int
    zone_letter : str

    def __init__(self, easting, northing, zone_number, zone_letter):
        self.easting = easting
        self.northing = northing
        self.zone_number = zone_number
        self.zone_letter = zone_letter

    def to_latlon(self) -> LatLongPoint:
        """
        Convert UTM coordinates to latitude and longitude.
        """
        latitude, longitude = utm.to_latlon(self.easting, self.northing, self.zone_number, self.zone_letter)
        return LatLongPoint(latitude, longitude) 

    def to_navsatfix_msg(self) -> NavSatFix:
        """
        Convert UTM coordinates to NavSatFix message.
        """
        lat_long = self.to_latlon()
        msg = NavSatFix()
        msg.latitude = lat_long.latitude
        msg.longitude = lat_long.longitude
        return msg
    
    def distance_to(self, other: 'UTMPoint') -> float:
        """
        Calculate the distance to another UTM point.
        """
        assert self.zone_number == other.zone_number, "Zone numbers must be the same for distance calculation"
        return math.dist((self.easting, self.northing), (other.easting, other.northing))

    def __repr__(self):
        return f"UTMPoint(x={self.x}, y={self.y}, zone_number={self.zone_number}, zone_letter={self.zone_letter})"