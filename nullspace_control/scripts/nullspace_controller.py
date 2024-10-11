# external imports
import numpy as np
from math import pi, cos, sin

class NullspaceController():
    # def __init__(self, Kg=5, Kf=3.5, Ko=1.4):
    def __init__(self, Kg=5, Kf=4, Ko=3):
    
        self.Kg = Kg
        self.Kf = Kf
        self.Ko = Ko

    def translation(self, x, y):
        """
        @param
        `x` -> Float indicating translation along x-axis
        `y` -> Float indicating translation along y-axis
                if goal_position:
    
        Matrix in homogeneous coordinates defining a 2d translation 
        """
        return np.matrix([[1, 0, x],
                          [0, 1, y],
                          [0, 0, 1]])

    def rotation(self, theta):
        """
        @param
        `theta` -> Float indicating rotation around z-axis
        
        @return
        Matrix in homogeneous coordinates defining a rotation around the z-axis
        """
        return np.matrix([[cos(theta), -sin(theta), 0],
                          [sin(theta),  cos(theta), 0],
                          [         0,           0, 1]])
    

    def get_move_to_goal_vector(self, goal_position):
        '''
        @Params
        * `goal_position` -> list of type [X, Y] or [] representing the center position of the legs pair

        @Return
        * `g` -> np.array shape column vector pointing towards the goal
        * `J` -> np.array Jacobian (row) vector pointing towards the goal
                    from math import pi, cos, sin ( numpy) and the Jacobian, a row vector (numpy).
        - If the goal_position is empty, return a (0, 0) column vector (numpy) and (0,0) Jacobian row vector (numpy).
        - E.g. format of the vectors:
        Direction vector (column): [[linear velocity]
                                    [rotational velocity]],
        Jacobian (row vector): [[linj, rotj]]
        '''

        if goal_position is None or not len(goal_position):
            vector_to_goal = np.array([[0],[0]])
            J = np.array([[0, 0]])
            return vector_to_goal, J
        
        x = goal_position[0]
        y = goal_position[1]
        distance_current = np.sqrt(x**2 + y**2)
        distance_desired = 0.5

        # calculate jacobian pointing to goal
        # Following slides formula J2 = [((x-xf) / distance(p, pf)), ((y-yf) / distance(p, pf)))
        linj = -x/distance_current
        rotj = -y/distance_current
        J = np.array([[linj, rotj]])
        
        # calculate vector pointing to goal
        # Following slides formula v2 = [[-xf/distance(p, pf)],    
        #                                [-yf/distance(p, pf)] ] * K2*(df-distance(p,pf))
        
        avoid_drift_term =  self.Kg * (distance_desired - distance_current)
        v = (linj) * avoid_drift_term
        omega = (rotj) * avoid_drift_term
        vector_to_goal = np.array([[v],
                                  [omega]])
        return vector_to_goal, J

    def get_formation_vector(self, positions, index, x, y, yaw):
        """
        @param
        `positions` -> List containing robot positions
        `index` -> Int index indicating a robot in a list of robot positions - robots start at 1,
                but with index we assumed that it is 
                an actual index (indexes start at 0 in python)
        `x` -> Float indicating positon of a robot on x-axis
        `y` -> Float indicating positon of a robot on y_axis
        `yaw` -> Float indication rotation of a robot around z-axis

        @return
        a column vector (numpy) for the robot to move into
        the correct position of the swarm
        """

        # Checks if positions is of length 3, if not, return a (0, 0) column vector (numpy)
        if len(positions) != 3:
            return np.zeros((2,1))
        
        positions = np.matrix(positions)
        centroid = np.mean(positions, axis=0)
        centroid_3_by_2 = np.tile(centroid,(3,1))

        # Calculates the matrix containing the direction vectors for the formation of each robot.
        d = 0.5 # desired distance
        desired_position = np.matrix([[              d,                0],
                                      [d*cos((2*pi)/3),  d*sin((2*pi)/3)],
                                      [d*cos((2*pi)/3), -d*sin((2*pi)/3)]])
        
        direction_vectors_4_all_robots = np.add(desired_position, centroid_3_by_2)
        direction_vector = direction_vectors_4_all_robots[index]

        # code below is taken from the example given in the assignment
        tran = self.translation(x,y)
        rot = self.rotation(yaw)

        transformation_matrix = np.matmul(tran, rot)
        
        world_coordinate = np.array([direction_vector[0, 0], direction_vector[0, 1], 1.0])

        robot_frame_of_reference_coordinate = np.matmul(np.linalg.pinv(transformation_matrix), world_coordinate)
        direction = self.Kf*robot_frame_of_reference_coordinate[0, 0:2]
        return np.array(direction.T)

    def get_obstacle_vector(self, obstacle_position):
        '''
        @param
        * `obstacle_position` -> list (empty [] or point [x,y] position of the closest obstacle)

        @return
        * `v` -> np column vector indicating the velocity and linear velocity needed to avoid an obstacle
        * `J` -> np row vector - jacobian of the `v` vector 

        @description
        - Calculates the obstacle vector according to the formula
        presented in the slides.
        - Returns a 2d column vector (numpy) and the Jacobian, a
        row vector (numpy)
        - If param obstacle_position is empty, return a (0, 0) column
        vector (numpy), and a (0, 0) Jacobian row vector (numpy)
        - Checks if the distance is larger than 0.4m, if so, return a (0,
        0) column vector (numpy), and a (0, 0) Jacobian row vector
        (numpy). Else calculate the obstacle avoidance vector and
        Jacobian using the formulas on the slides.
        - E.g. format of the vectors:
        Obstacle vector (column): [[linear velocity]
        [rotational velocity]],
        Jacobian (row vector): [[linj, rotj]]
        '''
        
        if len(obstacle_position) == 0:
            v = np.array([[0], [0]])
            J = np.array([[0, 0]])
            return v, J 
        
        # calculate euclidian distance to the robot
        x = obstacle_position[0]
        y = obstacle_position[1]
        distance = np.sqrt(x**2 + y**2)
        desired_distance = 0.4
        
        # we only care about obstacles closer than 0np.count_nonzero().4m
        if distance > desired_distance:
            v = np.array([[0], [0]])
            J = np.array([[0, 0]])
            return v, J 

        # calculate jacobian
        normalized_x_distance = -x/distance
        normalized_y_distance = -y/distance
        J = np.array([[normalized_x_distance, normalized_y_distance]])
        
        # calculate obstacle vector
        avoid_drift_term = self.Ko*(desired_distance - distance)
        v = J.T * avoid_drift_term
        return v, J

    def distance(self, point1, point2):
        return np.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)

    def get_direction_vector(self, goal_position, positions, index, x, y, yaw, obstacle_position):
        # calculate obstacle vector & Jacobian
        v1, J1 = self.get_obstacle_vector(obstacle_position)

        # calculate follow me vector & Jacobian
        v2, J2 = self.get_move_to_goal_vector(goal_position)
            
        # calculate formation control vector & Jacobian
        v3 = self.get_formation_vector(positions, index, x, y, yaw)

        # calculate vd(t) = v1 + (I - J1^-1 * J1) * [v2 + (I - J2^-1 * J2) * v3]
        I = np.identity(2)
        null1 = I - np.dot(np.linalg.pinv(J1), J1)
        null2 = I - np.dot(np.linalg.pinv(J2), J2)
        vd = v1 + np.matmul(null1, (v2 + np.matmul(null2,v3)))
        return vd