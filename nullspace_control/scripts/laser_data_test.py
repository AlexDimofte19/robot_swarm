import unittest
from laser_data import LaserData

class LaserDataTest(unittest.TestCase):
    def distance_test(self):
        laser_data = LaserData()
        point1 = [-7,-4]
        point2 = [17, 6.5]
        distance = laser_data.distance(point1, point2)
        distance_correct = 26.196374
        self.assertAlmostEqual(distance, distance_correct, msg='lol')


if __name__ == '__main__':
    unittest.main()