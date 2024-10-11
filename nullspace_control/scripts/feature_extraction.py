
# imports
import numpy as np
import math


class FeatureExtraction(object):

    def euclidian_distance(self, point1, point2):
        '''
        @Params
        `point1` -> list [x, y]
        `point2` -> list [x, y]

        @Return
        `distance` -> float 
        The Euclidean distance between param point1 and param point2

        @Description
        [Helper Function] Calculates the Euclidean distance between param `point1` and
        param `point2`, and returns this `distance`
        '''
        distance = (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 
        distance = math.sqrt(distance)
        return distance

    def get_girth(self, cluster):
        ''' 
        @Params
        `cluster` -> list (of points, assuming cluster
         contains at least 3 points)

        @Return
        `girth` -> float (indicating calculated girth of cluster)

        @Description
        The girth is calculated of the cluster, via the algorithm described in
        the paper. It returns the girth, a single float value.
        '''
        girth = [self.euclidian_distance(cluster[j], cluster[j+1])
                     for j, _ in enumerate(cluster[:-1])]
        
        girth = np.sum(girth)
        return girth

    def get_width(self, cluster):
        '''
        @Params
        `cluster` -> list (of points, assuming cluster
         contains at least 3 points)

        @Return
        `width` -> float (indicating calculated width of cluster)
        
        @Description
        The width is calculated of the cluster, via the algorithm described
        in the paper. It returns the width, a single float value.
        '''
        # get first and last points in the cluster
        first_point = cluster[0]
        last_point = cluster[-1]

        # width = euclidian distance(p_1, p_m)
        width = self.euclidian_distance(first_point, last_point)
        return width

    def get_intermediary_depth(self, cluster, j):
        p_first = np.array(cluster[0])
        p_end = np.array(cluster[-1])
        p_j = np.array(cluster[j])

        l_1 = p_end - p_first
        l_2 = p_j - p_first

        dot_11 = np.dot(l_1.T, l_1)
        dot_22 = np.dot(l_2.T, l_2)
        dot_12 = np.dot(l_1.T, l_2)
        
        dot_11 = float(1e-9) if dot_11 == 0 else dot_11 # avoid division by 0
        depth_2 = float((dot_22*dot_11 - dot_12**2))/float(dot_11)

        return np.sqrt(abs(depth_2))

    def get_depth(self, cluster):
        '''
        @Params
        `cluster` -> list (of points, assuming cluster
         contains at least 3 points)

        @Return
        `depth` -> float (indicating calculated depth of cluster)
        
        @Description
        The depth of the cluster is calculated, via the algorithm described
        in the paper, but square-rooted. It returns the depth, a single float
        value
        '''

        depth = 0
        for i in range(1, len(cluster)-1):
            intermediary_depth = self.get_intermediary_depth(cluster, i)
            if intermediary_depth > depth:
                depth = intermediary_depth

        return depth

    def get_features(self, cluster):
        '''
        @Params 
        `cluster` -> list (of points, contains at least 3 points, 
        e.g. this function should not be called (by the robot class) 
        with a cluster with less than 3 points in it
        
        @Return 
        features -> list with 3 features [girth, width, depth]

        @Description
        The 3 features are calculated and put in a list, in the following
        order; girth, width, depth. The list containing the features is
        returned. E.g., [girth, width, depth]
        '''
        girth = self.get_girth(cluster)
        width = self.get_width(cluster)
        depth = self.get_depth(cluster)
        features = [girth, width, depth]
        return features

