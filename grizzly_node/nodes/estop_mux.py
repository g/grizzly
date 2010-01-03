#!/usr/bin/python

# Controls mcu/estop based on input from system_estop or teleop_estop


import roslib; roslib.load_manifest('grizzly_node')
import rospy

from std_msgs.msg import Bool

class EstopWatch:
    def __init__(self):
        rospy.init_node('estop_watchdog')

        self.sysestop = False
        self.telestop = False

        rospy.Subscriber("system_estop",Bool,self.sysestop_callback)
        rospy.Subscriber("teleop_estop",Bool,self.telestop_callback)

        self.estop_pub = rospy.Publisher("mcu/estop",Bool)
        rospy.Timer(rospy.Duration(1/50.0), self.estop_watchdog)

        rospy.spin()

    def estop_watchdog(self,event):
        if (self.sysestop or self.telestop):
            self.estop_pub.publish(True)
        else:
            self.estop_pub.publish(False)

    def sysestop_callback(self,data):
        self.sysestop = data.data

    def telestop_callback(self,data):
        self.telestop = data.data

if __name__ == "__main__":
    EstopWatch()


