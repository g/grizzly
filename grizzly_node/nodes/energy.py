#!/usr/bin/python

# Node to estimate power usage!

import roslib; roslib.load_manifest('grizzly_node')
import rospy

from std_msgs.msg import Float32
from roboteq_msgs.msg import Feedback
from grizzly_msgs.msg import RawStatus
from math import exp

FR = 0
FL = 1
RR = 2
RL = 3
h_thresh = 50 #above this, assume battery is full
l_thresh = 46.5 #below this, assumed battery is dead

class EnergyEstimation:
    def __init__(self):
        rospy.init_node('energy_estimation')

        self.numbatpacks = rospy.get_param('numbatpacks',2)
        
        # Publishers & subscribers
        self.energy_pub = rospy.Publisher('mcu/energy', Float32)

        self.first_motor_volt_rxd = [False, False, False, False]
        self.first_user_volt_rxd = False
        self.first_motor_volts = [0,0,0,0]
        self.first_user_volt = 0

        curr_time = rospy.get_time()
        self.last_mot_times = [curr_time,curr_time,curr_time,curr_time]
        self.last_mot_watt = [0,0,0,0]

        self.last_user_time = curr_time
        self.last_user_watt = 0
        self.total_wattage_used = 0

        self.init_soc = 0

        self.full_bat_cap = 105*48*self.numbatpacks #105 A.h in each battery, x 48v for watt.H x number of battery packs(2 or 4) 

        # Timing
        self.rate = rospy.Rate(rospy.get_param('~hz',50))
        self.period = 1.0/rospy.get_param('~hz',50)

        rospy.Subscriber('mcu/status', RawStatus, self.HandleUserStatus)
        rospy.Subscriber('motors/front_right/feedback', Feedback, self.HandleFRFeedback)
        rospy.Subscriber('motors/front_left/feedback', Feedback, self.HandleFLFeedback)
        rospy.Subscriber('motors/rear_left/feedback', Feedback, self.HandleRLFeedback)
        rospy.Subscriber('motors/rear_right/feedback', Feedback, self.HandleRRFeedback)


        while (not self.process_first_voltage() and (not rospy.is_shutdown())):
            self.rate.sleep()


        total_wattage = Float32()
        while not rospy.is_shutdown():
            """ Main state machine loop """
            total_wattage.data = self.init_soc - self.total_wattage_used/float(self.full_bat_cap)
            self.energy_pub.publish(total_wattage)  
            self.rate.sleep()

    def HandleFRFeedback(self, data):
        self.process_mot_vi(data,FR) 
            
    def HandleFLFeedback(self, data):
        self.process_mot_vi(data,FL)

    def HandleRLFeedback(self, data):
        self.process_mot_vi(data,RL)

    def HandleRRFeedback(self, data):
        self.process_mot_vi(data,RR)

    # get current and voltage (power), integrate over time to get Watt.hours
    def HandleUserStatus(self,data):

        if (not self.first_user_volt_rxd):
            self.first_user_volt_rxd = True
            self.first_user_volt = data.voltage
        else:
            curr_t = rospy.get_time()
            curr_i = data.user_current
            curr_v = data.voltage
            curr_w = curr_i * curr_v
            dt = curr_t - self.last_user_time
            curr_wattage = dt * (curr_w + self.last_user_watt)/2.0
            curr_wattage = curr_wattage/3600 #convert to watt.hour
            self.total_wattage_used = self.total_wattage_used + curr_wattage
            self.last_user_watt = curr_w
            self.last_user_time = curr_t


    def process_mot_vi(self,data,motor_num):

        if (not self.first_motor_volt_rxd[motor_num]):
            self.first_motor_volt_rxd[motor_num] = True
            self.first_motor_volts[motor_num] = data.supply_voltage
        else:
            curr_t = rospy.get_time()
            curr_i = data.supply_current
            curr_v = data.supply_voltage
            curr_w = curr_i * curr_v
            dt = curr_t - self.last_mot_times[motor_num]
            curr_wattage = dt * (curr_w + self.last_mot_watt[motor_num])/2.0
            curr_wattage = curr_wattage/3600 # convert to watt.hour
            self.total_wattage_used = self.total_wattage_used + curr_wattage
            self.last_mot_watt[motor_num] = curr_w
            self.last_mot_times[motor_num] = curr_t

    def process_first_voltage(self):
        if (not (False in self.first_motor_volt_rxd) and self.first_user_volt_rxd): #get first estimate of SOC from voltages
            #initial voltage from motor controllers to low if Estop is pressed. ignore it
#            avg_meas_volt = sum(self.first_motor_volts)/float(len(self.first_motor_volts))
#            avg_meas_volt = (avg_meas_volt + self.first_user_volt)/2.0
            avg_meas_volt = self.first_user_volt
            if (avg_meas_volt >= h_thresh):
                self.init_soc = 1.000
            elif (avg_meas_volt <= l_thresh):
                self.init_soc = 0.000
            else:
                self.init_soc = self.get_soc_estimate(avg_meas_volt)

            
            return True
        else:
            return False
         
    def get_soc_estimate(self,voltage):
        #get soc charge estimate from voltage 
        a1 = 0.5256 
        b1 = 50.09  
        c1 = 0.0485 
        a2 = 0.8292  
        b2 = 51.94
        c2 = 2.822
        y = voltage
        x_inv = a1*exp(-pow((y-b1)/c1,2)) + a2*exp(-pow((y-b2)/c2,2));
        return x_inv

                
if __name__ == "__main__": 
    EnergyEstimation()
