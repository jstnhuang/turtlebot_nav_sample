#!/usr/bin/env python
from __future__ import division

import actionlib
import math
import rospy
import tf

from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RobotMover(object):
    def __init__(self, action_client):
        self._client = action_client
        self._client.wait_for_server()

    def move_to(self, frame, position, orientation, wait=False,
            done_callback=None, active_callback=None, feedback_callback=None):
        '''Sends the movement goal.

        Args:
            frame: The frame for which the position and orientation are
                specified. For example, if position=(1, 2, 0), then a 'map'
                frame would move to position (1, 2) on the map. A 'base_link'
                frame would move the robot 1 meter forward, and 2 meters to the
                left.
            position: The position to move to, as a geometry_msgs/Point
            orientation: The orientation of the robot, as a
                geometry_msgs/Quaternion
            wait: Whether to block until the action is done or not.
            done_callback: A callback f(state, result) to call when the action
                is done.
            active_callback: A callback f() to call when the action has started.
            feedback_callback: A callback f(feedback) to call while the robot is
                moving.
        '''
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.pose.position = position
        goal.target_pose.pose.orientation = orientation
        self._client.send_goal(
            goal,
            done_cb=done_callback,
            active_cb=active_callback,
            feedback_cb=feedback_callback
        )
        if wait:
            self._client.wait_for_result();

def done_callback(state, result):
    '''Prints the action's status when the movement is done.'''
    #See actionlib_msgs/GoalStatus for this enum definition.
    status_messages = ['Pending', 'Active', 'Preempted', 'Succeeded', 'Aborted',
        'Rejected', 'Preempting', 'Recalling', 'Recalled', 'Lost'
    ]
    rospy.loginfo('Done moving to goal.')
    rospy.loginfo('State: {}'.format(status_messages[state]))
    # result is empty.

def active_callback():
    '''Called when the goal becomes active (after it is sent).'''
    rospy.loginfo('Goal active.')

def feedback_callback(feedback):
    '''Prints the robot's current position while the movement is in progress.'''
    rospy.loginfo('Moving to goal. Feedback: {}'.format(feedback))
    pass

def to_quaternion(degrees):
    '''Returns a geometry_msgs/Quaternion corresponding to the given rotation.

    degrees is a rotation in a Cartesian plane.
    '''
    radians = degrees * math.pi / 180
    # The 's' in 'sxyz' means "static frame". 'xyz' tells the function what
    # order the arguments are in. For example, 'xyz' means that the first
    # arg is roll (rotation about the x-axis), the second arg is pitch, and
    # the last arg (most relevant for the Turtlebot) is yaw.
    #
    # quaternion_from_euler returns a numpy array instead of a ROS
    # geometry_msgs/Quaternion type, so we need to do the conversion.
    orientation_numpy = tf.transformations.quaternion_from_euler(
        0, 0, radians, 'sxyz')
    orientation = Quaternion(
        orientation_numpy[0],
        orientation_numpy[1],
        orientation_numpy[2],
        orientation_numpy[3]
    )
    return orientation

def main():
    '''Accepts user input and tells the robot to move somewhere on the map.'''
    rospy.init_node('move_robot')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    mover = RobotMover(client)
    while True:
        print 'Move the robot to the given pose. Press Ctrl+D to exit.'
        try:
            x = float(raw_input('X: '))
            y = float(raw_input('Y: '))
            degrees = float(raw_input('Rotation, in degrees: '))
        except ValueError:
            print 'Invalid input.'
            continue
        except EOFError:
            print
            return
        position = Point(x, y, 0)
        orientation = to_quaternion(degrees)

        mover.move_to('/map', position, orientation,
            wait=True,
            done_callback=done_callback,
            active_callback=active_callback,
            feedback_callback=feedback_callback
        )

if __name__ == '__main__':
    main()
