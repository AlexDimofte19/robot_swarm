
# external imports
from pickle import FALSE
import rospy
import numpy as np
import tf
import copy

# message imports
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# internal impoorts
from laser_data import LaserData
from feature_extraction import FeatureExtraction
from classifier import Classifier
from nullspace_controller import NullspaceController
from publish_markers import PublishMarkers


# Constants
CLUSTER_THRESHOLD = 0.09
LEG_SEPARATION = 0.2 #between the centers of legs
LEG_SEPARATION_ERROR = 0.05

# Data Collection
GATHERING_DATA = False
GETTING_LEGS_DATA = False
DATA_THRESHOLD = 10000

class Robot(object):
    
    def __init__(self, name):
        self.name = name
        self.laser_data = LaserData()
        self.feature_extraction = FeatureExtraction()
        self.classifier = Classifier()
        self.data_list = []
        self.distance_to_goal = np.inf
        self.is_leader = False

        self.cmd_vel_publisher = rospy.Publisher(name + "/cmd_vel", Twist, queue_size=1)
        self.leg_markers = PublishMarkers(frame_id = name+'/base_link', topic='legs')\
        
        # assignment 2
        self.prev_legs = None
        self.leg_pair_markers = PublishMarkers(frame_id = name+'/base_link', topic='leg_pair')
        self.nullspace_controller = NullspaceController()

        # assignment 3
        self.x = 0.0 
        self.y = 0.0 
        self.yaw = 0.0
        self.robot_positions = []
        self.odom_subscriber = rospy.Subscriber(name + "/base_pose_ground_truth", Odometry, callback=self.odom_callback, queue_size=1)
        
        # Laser subscriber initialized as last (AS ALWAYS)
        self.laser_subscriber = rospy.Subscriber(name + "/base_scan", LaserScan, callback=self.laser_scan_callback, queue_size=1)

    def get_distance_to_goal(self):
        return self.distance_to_goal

    def get_name(self):
        '''
        @params
        `None`

        @return
        Value of the member variable name of type 'str' value of the member variable name of type 'str'
        '''
        return str(self.name)

    def get_position(self):
        '''
        @params
        `None`

        @return
        A list, containing the member variables x, y in this order
        '''
        return [self.x, self.y]
    
    def update_robot_positions(self, robot_positions):
        """
        @params
        `robot_positions` -> list 

        @return
        A list, containing the member variables x, y in this order"""
        self.robot_positions = copy.deepcopy(robot_positions)
        

    def odom_callback(self, odom_msg):
        """
        @param
        `odom_msg` -> rospy message of type Odometry

        @description
        Sets the member variables x, y, yaw to the appropriate
        value gathered from the param odom_msg
        """
        #orientation
        orientation = odom_msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.yaw = yaw

        #postion
        position = odom_msg.pose.pose.position
        self.x = position.x
        self.y = position.y
    
    def get_closest_point(self, points):
        if len(points) == 0 or points is None:
            return []
        distances = [self.laser_data.distance(point, [0,0]) for point in points]
        idx = np.argmin(distances)
        return points[idx]

    def get_data(self, features):
        if GETTING_LEGS_DATA:
            for feature in features:
                self.data_list.append(np.array([np.array(feature), 1]))

            if len(self.data_list) > DATA_THRESHOLD:
                np.save("../../../../../data/legs1.npy", self.data_list )
                print("Saved legs")
                
        else:
            for feature in features:
                self.data_list.append(np.array([np.array(feature), 0]))

            if len(self.data_list) > DATA_THRESHOLD:
                np.save("../../../../../data/walls1.npy", self.data_list)
                print("Saved walls")
    
    def is_legs(self, separation, wide = False):
        """
        @param
        `separation` -> Float indicating distance between leg centroids
        `wide` -> Bool indicating whether to use the optimal leg separation ([0.15, 0.25]) or the largest possible separation ([0.1, 0.3])

        @return
        `Bool`

        @description
        Returns whether a given separation between leg centroids can possibly indicate legs or not
        """
        if wide:
            return LEG_SEPARATION - 2*LEG_SEPARATION_ERROR <= separation and separation <= LEG_SEPARATION + 2*LEG_SEPARATION_ERROR
        else:
            return LEG_SEPARATION - LEG_SEPARATION_ERROR <= separation and separation <= LEG_SEPARATION + LEG_SEPARATION_ERROR

    def limit_velocity(self, velocities):
        """
        @param
        * `velocities` -> column vector with the linear and rotational velocity (e.g. [[`v`], [`omega`]])

        @returns
        * `limited_velocities` -> `velocities` with `v` limited within [-0.2, 0.2] and `omega` limited within [-2, 2]
        """
        linear = [min(max(velocities[0][0], -0.2), 0.2)]
        rotational = [min(max(velocities[1][0], -2), 2)]
        return [linear, rotational]
    
    def get_and_mark_legs(self, features, filtered_clusters):
        predictions = self.classifier.predict(features)
        legs = []
        for i, pred in enumerate(predictions):
            if pred == 1:
                cluster = np.array(filtered_clusters[i])
                mean_X = np.mean(cluster.transpose()[0])
                mean_Y = np.mean(cluster.transpose()[1])
                legs.append([mean_X, mean_Y])

        # publish all found legs
        self.leg_markers.publish_markers(legs, r=0.0, g=1.0, b=0.0)
        return legs 

    def select_and_mark_optimal_leg_pair(self, clusters):
        filtered_clusters = self.laser_data.filter_clusters(clusters)
        features = [self.feature_extraction.get_features(cluster) for cluster in filtered_clusters]
        if len(features) == 0:
            return None
        # Data colletion
        if GATHERING_DATA:
            self.get_data(features)
            return None
        else:
            
            legs = self.get_and_mark_legs(features, filtered_clusters)

            # get all possible pairs of legs
            pairs = {}
            for idx, leg1 in enumerate(legs[:-1]):
                for leg2 in legs[idx+1:]:
                    pairs[self.laser_data.distance(leg1,leg2)] = list(np.mean([leg1,leg2], axis=0))
            
            # get array of leg separations
            filtered_leg_separations = [dist for dist in pairs if self.is_legs(dist)]
            
            # get optimal pairs of legs
            if len(filtered_leg_separations) == 1:
                #if only one pair detected select it
                optimal_legs = pairs[filtered_leg_separations[0]]

            elif len(filtered_leg_separations) > 1:
                #find pair which is closest to previous legs
                if self.prev_legs:
                    distance_to_prev_legs = {}
                    for separation in filtered_leg_separations:
                        distance_to_prev_legs[self.laser_data.distance(self.prev_legs, pairs[separation])] = separation
                    idx = np.argmin(list(distance_to_prev_legs.keys()))
                else:
                    #find pair which separation is closest to LEG_SEPARATION
                    idx = np.argmin(np.abs(LEG_SEPARATION - np.array(filtered_leg_separations)))
                optimal_legs = pairs[filtered_leg_separations[idx]]

            else:
                optimal_legs = None

            # publish selected pair of legs if any
            if optimal_legs:
                # change list of legs to the centroid of legs
                self.leg_pair_markers.publish_markers([optimal_legs],r=0.0, g=0.0, b=1.0)

            self.prev_legs = optimal_legs

            return optimal_legs

    def laser_scan_callback(self, laser_msg):
        '''
        @param
        `laser_msg` -> LaserScan

        @return
        `None`

        @description
        - Should convert the param laser_msg to cartesian data points (Part 1)
        - Should cluster the cartesian data points (Part 1)
        - (For this week's assignment) prints the number of clusters (Part 1)
        - (For this week's assignment) drives the robot forward with a
            velocity of 0.05m/s, and a rotational velocity of -0.3 rad/s (Part 1)
        - Should extract the features from the clusters (Part 2)
        - Should classify each cluster as a leg or no-leg (Part 3)
        - Publishes the mean point of each cluster that is classified as a leg,
            using the leg_markers, make the color green, by setting g = 1.0. (Part 3)
            Finds a pair of legs, using the previously classified legs.
        - Publish all the found legs as previously described
            (Assignment 1).
        - Publishes the center point of the pair of legs to follow via the
            leg_pair_markers, make the color blue, by setting b = 1.0.
        - Uses the null-space controller to get the direction vector for
            following the 'human'.
        - The robot should not exceed its speed limits.
        - The robot should follow the 'human' robot.
        '''
        
        # convert points to cartesian
        cartesian_data_points = self.laser_data.convert_to_cartesian(laser_msg)
        obstacle_position = self.get_closest_point(cartesian_data_points)

        # cluster the data points
        clusters = self.laser_data.get_clusters(cartesian_data_points, cluster_threshold=CLUSTER_THRESHOLD)
        optimal_legs = None
        if clusters:
            optimal_legs = self.select_and_mark_optimal_leg_pair(clusters)

        if optimal_legs:
            self.distance_to_goal = self.laser_data.distance(optimal_legs, [0,0])
        
        if not self.is_leader:
            optimal_legs = []

        index = int(self.name[-1])-1
        v = self.nullspace_controller.get_direction_vector(optimal_legs, self.robot_positions, index, self.x, self.y, self.yaw, obstacle_position)
        v, omega = self.limit_velocity(v)
        twist_msg = Twist()
        twist_msg.linear.x = v[0] 
        twist_msg.angular.z = omega[0] * 10 # scale rotational velocity by a factor of 10
        self.cmd_vel_publisher.publish(twist_msg)

