#!/usr/bin/python

# Converts incoming twist messages into outgoing drive messages

import roslib; roslib.load_manifest('grizzly_node')
import rospy

from geometry_msgs.msg import Twist
from roboteq_msgs.msg import Command
from roboteq_msgs.msg import Status


class MotionGenerator:
    def __init__(self):
        rospy.init_node('motion_generator')

        # Scale up angular rate to compensate for skid?
        #self.turn_scale = rospy.get_param('~turn_compensation', 1)


        # Width of our vehicle
        self.width = rospy.get_param('~vehicle_width',1.27)

        # 1 m/s equals what Roboteq float32 value?
        # 1000 / v_max (according to the Roboteq RPM max value)
        # Right now, Max RPM = 1500, which corresponds to 2.07345 m/s
        self.roboteq_scale = rospy.get_param('~roboteq_scale', 482.287)

        # Publishers & subscribers
        self.cmd_pub_fr = rospy.Publisher('motors/front_right/cmd', Command)
        self.cmd_pub_fl = rospy.Publisher('motors/front_left/cmd', Command)
        self.cmd_pub_rr = rospy.Publisher('motors/rear_right/cmd', Command)
        self.cmd_pub_rl = rospy.Publisher('motors/rear_left/cmd', Command)

        rospy.Subscriber('motors/front_right/status',Status, self.fr_statCallback)
        rospy.Subscriber('motors/front_left/status',Status, self.fl_statCallback)
        rospy.Subscriber('motors/rear_right/status',Status, self.rr_statCallback)
        rospy.Subscriber('motors/rear_left/status',Status, self.rl_statCallback)

        #Serious faults where every motor should turn off
        self.serious_fault = [Status.FAULT_OVERHEAT, Status.FAULT_OVERVOLTAGE, Status.FAULT_SHORT_CIRCUIT, Status.FAULT_MOSFET_FAILURE]


        #Assume there are no motor faults
        self.motor_fault = [False,False,False,False] #0-fr,1-fl,rr-2,rl-3

        rospy.Subscriber("safe_cmd_vel", Twist, self.callback)

        rospy.spin()

    def callback(self, data):
        """ Receive Twist message, do kinematics, output.
        Right now, use same speed for both wheels on one side """
        cmd = Twist()
        right_speed = data.linear.x + data.angular.z*self.width/2;
        left_speed = data.linear.x - data.angular.z*self.width/2;

        if (True in self.motor_fault): # make sure none of the motors are faulted
            #Turn off power to all motors, until fault is removed
            self.cmd_pub_fr.publish([int(0)])
            self.cmd_pub_fl.publish([int(0)])
            self.cmd_pub_rr.publish([int(0)])
            self.cmd_pub_rl.publish([int(0)])
        else:
            # Scale to whatever Roboteq needs
            self.cmd_pub_fr.publish([-int(right_speed*self.roboteq_scale)])
            self.cmd_pub_fl.publish([int(left_speed*self.roboteq_scale)])
            self.cmd_pub_rr.publish([-int(right_speed*self.roboteq_scale)])
            self.cmd_pub_rl.publish([int(left_speed*self.roboteq_scale)])

    #TODO: Combine into one callback? Callback data will need motor description
    def fr_statCallback(self, data):
        self.check_motor("Front Right",data.fault,0)

    def fl_statCallback(self, data):
        self.check_motor("Front Left",data.fault,1)

    def rr_statCallback(self, data):
        self.check_motor("Rear Right",data.fault,2)

    def rl_statCallback(self, data):
        self.check_motor("Rear Left",data.fault,3)

    #TODO: Log just once, instead of streaming the data out
    def check_motor(self,motor_string, status, motor_num): 
        if (status in self.serious_fault): # there is a serious motor fault, turn flag on and send user error 
            rospy.logerr(motor_string + " Motor Error:" + str(status))
            self.motor_fault[motor_num] = True
        elif (not self.motor_fault[motor_num]): #there is no fault, and the fault flag is on i.e. motor just came out of a fault
            self.motor_fault[motor_num] = False

if __name__ == "__main__": 
    MotionGenerator()
