#!/usr/bin/python

# Converts incoming twist messages into outgoing drive messages

import roslib; roslib.load_manifest('grizzly_node')
import rospy

from geometry_msgs.msg import Twist
from roboteq_msgs.msg import Command


class MotionGenerator:
    def __init__(self):
        rospy.init_node('motion_generator')

        # Scale up angular rate to compensate for skid?
        self.turn_scale = rospy.get_param('~turn_compensation', 1)

        # Width of our vehicle
        self.width = rospy.get_param('~vehicle_width')

        # 1 m/s equals what Roboteq float32 value?
        self.roboteq_scale = rospy.get_param('~roboteq_scale')

        # Publishers & subscribers
        self.cmd_pub_fr = rospy.Publisher('motors/front_right/cmd', Command)
        self.cmd_pub_fl = rospy.Publisher('motors/front_left/cmd', Command)
        self.cmd_pub_rr = rospy.Publisher('motors/rear_right/cmd', Command)
        self.cmd_pub_rl = rospy.Publisher('motors/rear_left/cmd', Command)

        rospy.Subscriber("safe_cmd_vel", Twist, self.callback)

        rospy.spin()

    def callback(self, data):
        """ Receive Twist message, do kinematics, output.
        Right now, use same speed for both wheels on one side """
        cmd = Twist()
        right_speed = data.linear.x + data.angular.z*self.width/2;
        left_speed = data.linear.x - data.angular.z*self.width/2;

        # Scale to whatever Roboteq needs
        self.cmd_pub_fr.publish([right_speed*self.roboteq_scale])
        self.cmd_pub_fl.publish([left_speed*self.roboteq_scale])
        self.cmd_pub_rr.publish([right_speed*self.roboteq_scale])
        self.cmd_pub_rl.publish([left_speed*self.roboteq_scale])

if __name__ == "__main__": 
    MotionGenerator()
