
# imports
import numpy as np
import math


class LaserData(object):
    def convert_to_cartesian(self, laser_msg):
        '''
        @Params
        `laser_msg` -> LaserScan

        @Return
        `points` -> list [x,y]
        list of points, where a point is a list of size 2 with parameters [x, y]

        @Description
        - All polar coordinates are converted to cartesian data points
        - Make sure you iterate over all the data points in ranges from the
        laser_msg.
        - If a distance equals range_max, ignore it, (the simulated laser
        always returns its range_max if nothing is detected, which is
        something a real laser doesn't do)
        - Points that are Inf or Nan are filtered out
        - Should return a list of cartesian data points, where a point is a list
        of size 2 with parameters [x, y],
        '''
        points = []
        min_range = laser_msg.range_min
        max_range = laser_msg.range_max

        angle = laser_msg.angle_min
        angle_increment = laser_msg.angle_increment

        ranges = laser_msg.ranges
        for r in ranges:
            if r != 'inf' and r >= min_range and r < max_range:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append(list([x, y]))
            angle += angle_increment
        return points


    def distance(self, point1, point2):
        '''
        @Params
        * `point1` -> list [x, y]
        * `point2` -> list [x, y]

        @Return
        * `distance` -> float 
        The Euclidean distance between param point1 and param point2

        @Description
        * Calculates the Euclidean distance between param `point1` and
        param `point2`, and returns this `distance`
        '''
        distance = (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 
        distance = math.sqrt(distance)
        return distance
        
    def filter_clusters(self, clusters):
        return [cluster for cluster in clusters if len(cluster) >= 3]


    def get_clusters(self, data, cluster_threshold=0.05):
        '''
        @Param
        `data` -> list of points [x,y]
        `cluster_threshold` -> float > 0.0 (default=0.05) 
        
        @Return
        `clusters` -> list
        list of clusters, where a cluster is a list of points, where a point is
        a list of size 2 with parameters [x, y]
        
        @Description
        - Clusters the data into clusters, two points belong to the same
        cluster if the distance between two points is smaller or equal then
        param cluster_threshold
        - A cluster can contain one or more points
        - All points in data are clustered
        - Should return a list of clusters, where a cluster is a list of points,
        where a point is a list of size 2 with parameters [x, y] e.g.:
        [ [ [x1,y1], [x2, y2] ], [ [x3, y3], [x4, y4], [x5, y5] ] ], this is 2 clusters,
        1 cluster contains point 1 and 2, the other cluster contains points
        3, 4 and 5.
        - data is given to the member variable model's predict function,
        and its return values are returned. The return value will be a list of
        equal length of the amount of items in data, containing the
        classification predictions.
        '''
        if not data:
            return []

        prev_point = data[0]
        clusters = []
        cluster = []
        cluster.append(prev_point)
        for i, point in enumerate(data[1:]):
            distance = self.distance(prev_point, point)
            if distance > cluster_threshold:
                clusters.append(cluster)
                cluster = []

            cluster.append(point)
            prev_point = point

        if clusters:
            last_distance = self.distance(cluster[-1], clusters[0][0])
            if last_distance <= cluster_threshold:
                clusters[0] = cluster + clusters[0]
                return clusters

        clusters.append(cluster)
        return clusters

