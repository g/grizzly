#!/usr/bin/python

# Fan control. At the moment, full power!

import roslib; roslib.load_manifest('grizzly_node')
import rospy

from std_msgs.msg import Bool

class FanControl:
    def __init__(self):
        rospy.init_node('fan_control')

        # Publishers & subscribers
        self.cmd_fan = rospy.Publisher('/mcu/fan', Bool)

        # Timing
        self.rate = rospy.Rate(rospy.get_param('~hz',50))
        self.period = 1.0/rospy.get_param('~hz',50)

        while not rospy.is_shutdown():
            """ Main state machine loop """
            self.cmd_fan.publish(Bool(True))
            self.rate.sleep()

if __name__ == "__main__": 
    FanControl()
