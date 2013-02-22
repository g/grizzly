#!/usr/bin/python

# Fan control. At the moment, full power!

import roslib; roslib.load_manifest('grizzly_node')
import rospy

from std_msgs.msg import Bool
from roboteq_msgs.msg import Status

FR = 0
FL = 1
RR = 2
RL = 3

class FanControl:
    def __init__(self):
        rospy.init_node('fan_control')
        
        self.safe_motor_temp = rospy.get_param('safe_motor_temp',60)
        self.safe_ch_temp = rospy.get_param('safe_ch_temp',50)
        self.safe_ic_temp = rospy.get_param('safe_ic_temp',50)
        self.hyst_size = rospy.get_param('hyst_size',10)

        self.fan_state = False

        self.channel1_temp = [0,0,0,0]
        self.channel2_temp = [0,0,0,0]
        self.ic_temp = [0,0,0,0]
        self.motor_temp = [0,0,0,0]


        # Timing
        self.rate = rospy.Rate(rospy.get_param('~hz',10))
        self.period = 1.0/rospy.get_param('~hz',10)

        # Publishers & subscribers
        self.cmd_fan = rospy.Publisher('mcu/fan', Bool)
        rospy.Subscriber('motors/front_right/status', Status, self.HandleFRStatus)
        rospy.Subscriber('motors/front_left/status', Status, self.HandleFLStatus)
        rospy.Subscriber('motors/rear_left/status', Status, self.HandleRLStatus)
        rospy.Subscriber('motors/rear_right/status', Status, self.HandleRRStatus)



        while not rospy.is_shutdown():
            """ Main state machine loop """
            self.check_temps()
            self.cmd_fan.publish(bool(self.fan_state))
            self.rate.sleep()

    def HandleFRStatus(self, data):
        self.process_temps(data,FR) 

    def HandleFLStatus(self, data):
        self.process_temps(data,FL)

    def HandleRLStatus(self, data):
        self.process_temps(data,RL)

    def HandleRRStatus(self, data):
        self.process_temps(data,RR)

    def process_temps(self,data,motor_num):
        self.motor_temp[motor_num] = data.motor_temperature[0]
        self.channel1_temp[motor_num]  = data.channel_temperature[0]
        self.channel2_temp[motor_num] = data.channel_temperature[1]
        self.ic_temp[motor_num] = data.ic_temperature
        
    def check_temps(self):
        motor_safe = ic_safe = ch_safe = 0

        #TODO:Is there a cleaner way of doing this check?
        for i in range(4):
            #Check if any temps are above safe temp 
            if (self.motor_temp[i] > self.safe_motor_temp ):
               self.fan_state = True
               break
           
            if (self.ic_temp[i] > self.safe_ic_temp ):
               self.fan_state = True
               break

            if (self.channel1_temp[i] > self.safe_ch_temp ):
               self.fan_state = True
               break

            #Count the number of motors, ic's and channels that are safe
            if (self.motor_temp[i] < (self.safe_motor_temp - self.hyst_size)):
                motor_safe = motor_safe + 1 

            if (self.ic_temp[i] < (self.safe_ic_temp - self.hyst_size)):
                ic_safe = ic_safe + 1

            if (self.channel1_temp[i] < (self.safe_ch_temp - self.hyst_size)):
                ch_safe = ch_safe + 1  

        #if all motors,ics and channels are fine, turn off fan
        if (motor_safe > 3 and ic_safe > 3 and ch_safe > 3):
            self.fan_state = False

if __name__ == "__main__": 
    FanControl()
