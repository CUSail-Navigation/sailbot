import random

class FakeRadio:
    def __init__(self):
        pass

    def read_data(self):
        # Generate random data within the expected range
        # a = random.randint(-90, 90)
        # b = random.randint(-30, 30)
        return 10, 10

    def close(self):
        pass  # No cleanup needed for fake data
