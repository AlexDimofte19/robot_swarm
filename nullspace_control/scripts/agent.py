
# imports
import rospy
import numpy as np
from robot import Robot

class Agent(object):
    def __init__(self):
        self.robots = [Robot(name = "robot_1"), 
                       Robot(name = "robot_2"),
                       Robot(name = "robot_3")]
        self.timer = self._timer = rospy.Timer(rospy.Duration(1.0/20.0), self.timer_callback)
    
    def timer_callback(self, event=None):
        """
        @param
        `event` not used but required, just in case we need to call it manually it is set to default - None

        @description
        Gathers all the positions (x, y) for each robot using the
        get_position function, appends these positions to a list, and
        gives each robot this list of positions, using the
        update_robot_positions function
        """
        # get positions
        positions = [robot.get_position() for robot in self.robots]
        # update positions
        distances = []
        for robot in self.robots:
            robot.update_robot_positions(positions)
            distances.append(robot.distance_to_goal)
            robot.is_leader = False
        idx_leader = np.argmin(distances)
        self.robots[idx_leader].is_leader = True

if __name__ == "__main__":
    rospy.init_node("agent")
    agent = Agent()
    rospy.spin()