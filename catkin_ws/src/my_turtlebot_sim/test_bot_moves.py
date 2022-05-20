#!/usr/bin/env python

import math
import unittest
import rospy
from gazebo_msgs.msg import ModelStates


class TestBotMoves(unittest.TestCase):
    def test_movement(self):
        rospy.init_node('test_movement', log_level=rospy.DEBUG)

        self.idx = None
        self.travelled = 0.0

        # We're expecting the robot to move this many meters from the origin
        expected_travel = 1.5

        # Subscribe to Gazebo model states, so we can check if our robot moved.
        self.subscriber = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.movement_callback)

        # Keep iterating until the robot travels as far as we expect
        while self.travelled < expected_travel and not rospy.is_shutdown():
            rospy.loginfo('Travelled; %s m', round(self.travelled, 1))
            rospy.sleep(1)

        # The robot has traveled as far as we expect. Test complete!
        assert self.travelled >= expected_travel

    def movement_callback(self, data):
        # Find the index of the turtlebot model within all Gazebo models.
        if self.idx is None:
            self.idx = 0
            for name in data.name:
                if name == 'turtlebot3_burger':
                    break
                self.idx += 1

        # Save turtlebots current position
        x = data.pose[self.idx].position.x
        y = data.pose[self.idx].position.y

        # Use pythagoras to get total distance travelled from the origin
        self.travelled = math.sqrt(abs(x) ** 2 + abs(y) ** 2)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('my_turtlebot_sim', 'test_bot_moves', TestBotMoves)
