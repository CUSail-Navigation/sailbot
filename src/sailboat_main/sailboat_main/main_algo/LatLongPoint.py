import UTMPoint

class LatLongPoint():
    """
    A class to represent a point in latitude and longitude coordinates.
    """
    latitude : float
    longitude : float
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def to_utm(self) -> 'UTMPoint':
        """
        Convert latitude and longitude to UTM coordinates.
        """
        x, y, zone_number, zone_letter = utm.from_latlon(self.latitude, self.longitude)
        return UTMPoint(x, y, zone_number, zone_letter) 

    def __repr__(self):
        return f"latitude={self.latitude}, longitude={self.longitude}"