#!/usr/bin/python

# Converts incoming twist messages into outgoing drive messages
# Monitors for motor and encoder safety

import roslib; roslib.load_manifest('grizzly_node')
import rospy
import math

from geometry_msgs.msg import Twist
from roboteq_msgs.msg import Command
from roboteq_msgs.msg import Status,Feedback
from grizzly_msgs.msg import RawStatus
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

#motor indices
FR = 0
FL = 1
RR = 2
RL = 3

#diag msg types
MOTOR_FAULT_DIAG = 0
MOTOR_COMM_DIAG = 1
MCU_COMM_DIAG = 2
ESTOP_DIAG = 3
ENC_DIAG = 4

class MotionGenerator:
    def __init__(self):
        rospy.init_node('motion_generator')

        # Scale up angular rate to compensate for skid?
        #self.turn_scale = rospy.get_param('~turn_compensation', 1)

        #Encoder error watchdog parameters
        self.enc_watchdog_period = rospy.get_param('enc_watchdog_period',1/10.0) #default 10hz
        self.enc_error_time = rospy.get_param('enc_error_time',0.5)
        self.enc_error_thresh = rospy.get_param('enc_error_thresh',75)
        self.enc_tics_diff_thresh = rospy.get_param('enc_tics_diff_thresh',8)

        # Vehicle Parameters
        self.width = rospy.get_param('~vehicle_width',1.01)
        self.gear_down = rospy.get_param('~gearing', 50.0)
        self.wheel_radius = rospy.get_param('~wheel_radius',0.333)
        self.max_rpm = rospy.get_param('~max_rpm',3500.0) #max command (1000) sent to roboteq attains this RPM value

               
        # 1 m/s equals how many RPMs at the wheel?
        rpm_scale = 1
        rpm_scale /= (2*math.pi*self.wheel_radius) #convert m/s to rotations at the wheel
        rpm_scale *= self.gear_down #convert rotations at the wheel to rotations at the motor
        rpm_scale *= 60 # seconds to mins

        #Convert rpms to roboteq input units (1000 units to get to max rpm)
        calc_scale = rpm_scale *  (1000.0/self.max_rpm)
        self.roboteq_scale = rospy.get_param('~roboteq_scale',calc_scale)
        self.mcu_watchdog_time = rospy.get_param('~mcu_watchdog_time',0.5)
        self.mot_watchdog_time = rospy.get_param('~mot_watchdog_time',2)


        # Publishers & subscribers
        self.cmd_pub_fr = rospy.Publisher('motors/front_right/cmd', Command)
        self.cmd_pub_fl = rospy.Publisher('motors/front_left/cmd', Command)
        self.cmd_pub_rr = rospy.Publisher('motors/rear_right/cmd', Command)
        self.cmd_pub_rl = rospy.Publisher('motors/rear_left/cmd', Command)
        self.cmd_estop = rospy.Publisher('system_estop', Bool)

        #Serious faults where every motor should turn off
        self.serious_fault = [Status.FAULT_OVERHEAT, Status.FAULT_OVERVOLTAGE, Status.FAULT_SHORT_CIRCUIT, Status.FAULT_MOSFET_FAILURE]

        self.mcu_heartbeat_rxd = False 
        self.mcu_dead = False
        self.estop_status = RawStatus.ERROR_ESTOP_RESET

        
        self.mot_setting = [0,0,0,0]
        self.encreading = [0,0,0,0]
        self.enc_tics_in_error = [0,0,0,0] #track number of consecutive error measurements that were too large
        self.encoder_fault = [False, False, False, False] #assume encoders are okay
        self.num_enc_fault = [0,0,0,0] #track consecutive encoder faults
        self.mot_heartbeat_rxd = [False, False, False, False]
        self.mot_node_dead = [True, True, True, True]


        #Assume there are no motor faults
        self.motor_fault = [False,False,False,False] #0-fr,1-fl,rr-2,rl-3

        self.motion_diag = MotionDiag()

        rospy.Subscriber('motors/front_right/status',Status, self.fr_statCallback)
        rospy.Subscriber('motors/front_left/status',Status, self.fl_statCallback)
        rospy.Subscriber('motors/rear_right/status',Status, self.rr_statCallback)
        rospy.Subscriber('motors/rear_left/status',Status, self.rl_statCallback)
        
        rospy.Subscriber('motors/front_right/feedback',Feedback, self.fr_fbCallback)
        rospy.Subscriber('motors/front_left/feedback',Feedback, self.fl_fbCallback)
        rospy.Subscriber('motors/rear_right/feedback',Feedback, self.rr_fbCallback)
        rospy.Subscriber('motors/rear_left/feedback',Feedback, self.rl_fbCallback)

        rospy.Subscriber('mcu/status',RawStatus,self.mcu_statCallback)

        rospy.Timer(rospy.Duration(self.enc_watchdog_period), self.encoder_watchdog)
        rospy.Timer(rospy.Duration(self.mcu_watchdog_time), self.mcu_watchdog)
        rospy.Timer(rospy.Duration(self.mot_watchdog_time), self.mot_watchdog)
        
        rospy.Subscriber("safe_cmd_vel", Twist, self.callback)

        rospy.spin()

    def callback(self, data):
        """ Receive Twist message, do kinematics, output.
        Right now, use same speed for both wheels on one side """
        cmd = Twist()
        right_speed = data.linear.x + data.angular.z*self.width/2;
        left_speed = data.linear.x - data.angular.z*self.width/2;

        # Scale to whatever Roboteq needs
        self.mot_setting[FR] = -right_speed * self.roboteq_scale
        self.mot_setting[FL] = left_speed * self.roboteq_scale
        self.mot_setting[RR] = -right_speed * self.roboteq_scale
        self.mot_setting[RL] = left_speed * self.roboteq_scale
         
         
        #Dont send the command if 
        #a) Motor is faulted (incl. encoder issues)
        #b) Motor node is dead
        #c) Mcu node is dead
        #d) Estop is not cleared and/or pre-charge is not completed)
        #e) There is an encoder fault

        #if ((True in self.motor_fault) or (True in self.mot_node_dead) or self.mcu_dead or (self.estop_status!=0) or (True in self.encoder_fault):
        if ((True in self.motor_fault) or (True in self.mot_node_dead) or self.mcu_dead or (self.estop_status!=0)):
            #Turn off power to all motors, until fault is removed
            self.cmd_pub_fr.publish([int(0)])
            self.cmd_pub_fl.publish([int(0)])
            self.cmd_pub_rr.publish([int(0)])
            self.cmd_pub_rl.publish([int(0)])
        else:
            self.cmd_pub_fr.publish([int(self.mot_setting[FR])])
            self.cmd_pub_fl.publish([int(self.mot_setting[FL])])
            self.cmd_pub_rr.publish([int(self.mot_setting[RR])])
            self.cmd_pub_rl.publish([int(self.mot_setting[RL])])
            

    def mcu_statCallback(self,data):
        self.mcu_heartbeat_rxd = True
        self.estop_status = data.error 

    def mcu_watchdog(self, event):
        if (not self.mcu_heartbeat_rxd):
            self.mcu_dead = True
            err_str = "MCU Communication is inactive. Vehicle has been deactivated. Please reset systems"
            rospy.logerr(err_str)
            self.motion_diag.setup_publish_diag(MCU_COMM_DIAG,DiagnosticStatus.ERROR, err_str, rospy.get_rostime())
        else:
            ok_str = "MCU Communication is active and OK"
            self.mcu_heartbeat_rxd = False
            self.mcu_dead = False
            self.motion_diag.setup_publish_diag(MCU_COMM_DIAG,DiagnosticStatus.OK, ok_str, rospy.get_rostime())


        #if pre charge status is activated, keep count
        if self.estop_status == RawStatus.ERROR_BRK_DET:
            self.pre_charge_timeout+=1
        else:
            self.pre_charge_timeout = 0

        #if precharge status is active for more than 4 seconds, fire estop. dont reset
        if self.pre_charge_timeout > (4/(self.mcu_watchdog_time)):
            self.cmd_estop.publish(True)
            error_str = "Precharge malfunction. Estop activated. Please reboot all systems"
            rospy.logerr(error_str)
            self.motion_diag.setup_publish_diag(ESTOP_DIAG, DiagnosticStatus.ERROR, error_str,rospy.get_rostime())
        elif self.estop_status!=0:
            warn_str = "EStop is activated. Vehicle will not move"
            rospy.logwarn(warn_str)
            self.motion_diag.setup_publish_diag(ESTOP_DIAG, DiagnosticStatus.WARN, warn_str, rospy.get_rostime())
        else:
            ok_str = "EStop System Open and Functional"
            self.motion_diag.setup_publish_diag(ESTOP_DIAG, DiagnosticStatus.OK, ok_str, rospy.get_rostime())


     
    #TODO: Combine into one callback? Callback data will need motor description
    def fr_statCallback(self, data):
        self.check_motor(data.fault,0)
        self.mot_heartbeat_rxd[FR] = True

    def fl_statCallback(self, data):
        self.check_motor(data.fault,1)
        self.mot_heartbeat_rxd[FL] = True

    def rr_statCallback(self, data):
        self.check_motor(data.fault,2)
        self.mot_heartbeat_rxd[RR] = True

    def rl_statCallback(self, data):
        self.check_motor(data.fault,3)
        self.mot_heartbeat_rxd[RL] = True

    def mot_watchdog(self, event): 
        if (False in self.mot_heartbeat_rxd): #its been 0.5 seconds since we've received data from the motor controllers, kill all motion
            f_index = self.mot_heartbeat_rxd.index(False )
            self.mot_node_dead[f_index] = True
            error_str = self.get_motor_string(f_index) + " motor controller communication is inactive. Vehicle has been deactivated. Please reboot systems"
            rospy.logerr(error_str)
            self.motion_diag.setup_publish_diag(MOTOR_COMM_DIAG, DiagnosticStatus.ERROR,error_str,rospy.get_rostime())
        else:
            self.mot_heartbeat_rxd = [False, False, False, False]
            self.mot_node_dead = [False, False, False, False]
            ok_str = "Motor communication is active and functioning"
            self.motion_diag.setup_publish_diag(MOTOR_COMM_DIAG, DiagnosticStatus.OK,ok_str,rospy.get_rostime())

    def fr_fbCallback(self, data):
        self.encreading[FR] = data.encoder_rpm[0]

    def fl_fbCallback(self, data):
        self.encreading[FL] = data.encoder_rpm[0]

    def rr_fbCallback(self, data):
        self.encreading[RR] = data.encoder_rpm[0]

    def rl_fbCallback(self, data):
        self.encreading[RL] = data.encoder_rpm[0]

    #TODO: Log at X frequency, instead of streaming the logs out
    def check_motor(self, status, motor_num): 
        if (status in self.serious_fault): # there is a serious motor fault, turn flag on and send user error 
            error_str = self.get_motor_string (motor_num)+ " Motor Error:" + str(status)
            rospy.logerr(error_str)
            self.motor_fault[motor_num] = True
            self.motion_diag.setup_publish_diag(MOTOR_FAULT_DIAG, DiagnosticStatus.OK, error_str, rospy.get_rostime()) 
        elif (not self.motor_fault[motor_num]): #there is no fault, and the fault flag is on i.e. motor just came out of a fault
            self.motor_fault[motor_num] = False
            ok_str = "Motor Faults Cleared"
            self.motion_diag.setup_publish_diag(MOTOR_FAULT_DIAG, DiagnosticStatus.OK, ok_str,rospy.get_rostime()) 



    def encoder_watchdog(self, event):
        for i in range(0,4):
            error = math.fabs(self.mot_setting[i]*(self.max_rpm/1000.0) - self.encreading[i])

            if (error > self.enc_error_thresh):
                self.enc_tics_in_error[i]+=1
            else:
                self.enc_tics_in_error[i]=0
                self.num_enc_fault[i]=0

            if (self.enc_tics_in_error[i] * self.enc_watchdog_period >= self.enc_error_time):
                if (self.assess_encoder_fault(i)):
                    self.num_enc_fault[i]+=1
                    if (self.num_enc_fault[i] > 3):
                        self.encoder_fault[i] = True

        if (True in self.encoder_fault):
            t_index = self.encoder_fault.index(True)
            error_string = "Encoder Fault on the " + self.get_motor_string(t_index) + " Motor" 
            self.motion_diag.setup_publish_diag(ENC_DIAG, DiagnosticStatus.ERROR, error_string, rospy.get_rostime())
            #self.cmd_estop.publish(True)
        else:
            ok_string = "Encoder system is active and functional"
            self.motion_diag.setup_publish_diag(ENC_DIAG, DiagnosticStatus.OK, ok_string, rospy.get_rostime())

    def assess_encoder_fault(self, mot_num): #function to check if the other motors (other than mot_num) also have similar levels of encoder errors
        err_motor_count = 3 #assume all other motors have large errors as well and therefore the error in the current motors is not a problem
        for i in range(0,4): #go through all the motors
            if (i!=mot_num): #except the one in question
                enc_diff = math.fabs(self.enc_tics_in_error[mot_num] - self.enc_tics_in_error[i]) #calculate the difference in the tics in error
                if (enc_diff > self.enc_tics_diff_thresh): #if the difference is over a certain value, decrement the number of motors
                    err_motor_count-=1
        
        if (err_motor_count == 0): #if > 0 then there is atleast one other motor behaving the same way as this motor (probably not an encoder fault) 
            return True
        else:
            return False

    def get_motor_string(self, mot):
        return {
            FR: 'Front Right',
            FL: 'Front Left',
            RR: 'Rear Right',
            RL: 'Rear Left' 
        }.get(mot,'Unknown')


class MotionDiag:
    def __init__(self):
      
        self.diag_motor_fault = []
        self.diag_motor_node_dead = []
        self.diag_mcu_node_dead = []
        self.diag_estop_status = []
        self.diag_enc_status = []

        self.last_diagnostics_time = rospy.get_rostime() 

        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
   
    def setup_publish_diag(self, diag_msg, diag_status, diag_string, time):
        if (diag_msg==MOTOR_FAULT_DIAG): #motor fault message update
            self.diag_motor_fault = DiagnosticStatus(name="System Motor Status", level=diag_status, message=diag_string) 
        elif (diag_msg==MOTOR_COMM_DIAG): #motor nodes status
            self.diag_motor_dead = DiagnosticStatus(name="System Motor Comm Status", level=diag_status, message=diag_string)
        elif (diag_msg==MCU_COMM_DIAG): #mcu comm status
            self.diag_mcu_node_dead = DiagnosticStatus(name="System MCU Comm Status", level=diag_status, message=diag_string)
        elif (diag_msg==ESTOP_DIAG): #estop status
            self.diag_estop_status = DiagnosticStatus(name="System EStop Status", level=diag_status, message=diag_string)
        elif (diag_msg==ENC_DIAG): #encoder diagnostics
            self.diag_enc_status = DiagnosticStatus(name="System Encoder Status", level=diag_status, message = diag_string)
        
        if (time - self.last_diagnostics_time).to_sec() < 1.0:
            return

        self.last_diagnostics_time = time
        
        diag = DiagnosticArray()
        diag.header.stamp = time
        if self.diag_motor_fault:
            diag.status.append(self.diag_motor_fault)

        if self.diag_motor_node_dead:
            diag.status.append(self.diag_motor_node_dead)

        if self.diag_mcu_node_dead:
            diag.status.append(self.diag_mcu_node_dead)

        if self.diag_estop_status:
            diag.status.append(self.diag_estop_status)

        if self.diag_enc_status:
            diag.status.append(self.diag_enc_status)
        
        self.diag_pub.publish(diag)

if __name__ == "__main__": 
    MotionGenerator()
