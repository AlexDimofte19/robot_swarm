import unittest
import numpy as np
from feature_extraction import FeatureExtraction

feature_extraction = FeatureExtraction()
point_1 = [1,0]
point_2 = [1,1]
point_3 = [0,1]
cluster = [point_1, point_2, point_3]
cluster2 = [[0,-2],[-1,-2],[-2,-2],[-2,-1],[-2,0]]
cluster3 = [[0,-2],[1,-2],[2,-2],[2,-1],[2,0]]
cluster4 = [[0,2],[-1,2],[-2,2],[-2,1],[-2,0]]

class TestFeatureExtractor(unittest.TestCase):
    def test_euclidian_distance(self):
        true_dist = 1
        test_dist = feature_extraction.euclidian_distance([0,0], [0,1])
        self.assertAlmostEqual(true_dist, test_dist)
    
    def test_width(self):
        true_width = np.sqrt(2)
        test_width = feature_extraction.get_width(cluster)
        self.assertAlmostEqual(true_width, test_width)

    def test_girth(self):
        true_girth = 2
        test_girth = feature_extraction.get_girth(cluster)
        self.assertAlmostEqual(true_girth, test_girth)

    def test_depth(self):
        true_depth = np.sqrt(2)/2
        test_depth = feature_extraction.get_depth(cluster)
        self.assertAlmostEqual(true_depth, test_depth)
    
    def test_depth2(self):
        true_depth2 = np.sqrt(2)
        test_depth2 = feature_extraction.get_depth(cluster2)
        self.assertAlmostEqual(true_depth2, test_depth2)
        
    def test_depth3(self):
        true_depth3 = np.sqrt(2)
        test_depth3 = feature_extraction.get_depth(cluster3)
        self.assertAlmostEqual(true_depth3, test_depth3)
    
    def test_depth4(self):
        true_depth4 = np.sqrt(2)
        test_depth4 = feature_extraction.get_depth(cluster4)
        self.assertAlmostEqual(true_depth4, test_depth4)


    

unittest.main() 