#!/usr/bin/python

# Delays incoming twist messages, ensures that they ramp smoothly
# Sets off beeper during delay interval and as appropriate during motion

#
# Logic:
#
# Republish cmd_vel to safe_cmd_vel according to the independent
# translational and acceleration ramps. 
#
# If any of the following conditions are met, or have been in the last X 
# seconds, do not publish at all:
# - We have not received cmd_vel for >10 s
# - We are seeing an error >1 on the status from the AVR (error=1 is a timeout)
# - We have just started up
#
# When we are republishing, pulse beacon and turn headlights to solid
# When we are in the X second startup phase, pulse beacon faster, pulse
# beeper, and flash head and taillamps
# When we are going backwards, turn taillamps to solid
#

import roslib; roslib.load_manifest('grizzly_node')
import rospy
from math import copysign

from geometry_msgs.msg import Twist
from grizzly_msgs.msg import RawStatus, Ambience


class MotionManager:
    def __init__(self):
        rospy.init_node('motion_manager')

        # Delay time [s]
        self.motion_delay = rospy.get_param('~motion_delay', 2)
   
        # Max trans speed [m/s]
        self.trans_speed = rospy.get_param('~trans_speed', 2)
 
        # Max rot speed [rad/s]
        self.rot_speed = rospy.get_param('~rot_speed', 2)
 
        # Max trans acceleration [m/s^2]
        self.trans_accel = rospy.get_param('~trans_accel', 0.5)
    
        # Max rot acceleration [rad/s^2]
        self.rot_accel = rospy.get_param('~rot_accel', 0.5)

        # Command timeout [s]
        self.no_cmd_timeout = rospy.get_param('~no_cmd_timeout', 0.1)

        # Motion timeout [s]
        self.no_motion_timeout = rospy.get_param('~no_motion_timeout', 10)

        # No-motion value [m/s & rad/s]
        self.no_motion_value = rospy.get_param('~no_motion_value',0.01)

        # Publishers & subscribers
        self.cmd_vel = rospy.Publisher('safe_cmd_vel', Twist)

        self.cmd_lights = rospy.Publisher('mcu/ambience', Ambience)

        rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("mcu/status", RawStatus, self.status_callback)

        # Timing
        self.rate = rospy.Rate(rospy.get_param('~hz',50))
        self.period = 1.0/rospy.get_param('~hz',50)

        # Time at which the last command was received [s]
        self.last_cmd_time = 0

        # Time at which the last command with a nonzero motion was received [s]
        self.last_motion_time = 0

        # Last reference setpoint received
        self.cmd_ref = Twist()

        # Last command sent
        self.cmd = Twist() 
    
        # Time at which we last had an error
        self.last_error_time = 0 

        # Explicit errors?
        self.control_error = False

        # Start main state machine
        self.loop()

    def loop(self):

        # State variable
        # 0 = Error/startup state
        # 1 = No error, but still counting down
        # 2 = Normal behaviour (ramped velocity, lights)
        motion_state = 0

        while not rospy.is_shutdown():
            """ Main state machine loop """
            if motion_state == 0:
                # State: 
                # Update last error time
                self.last_error_time = rospy.get_time()
                # Reset last command setpoint
                self.cmd = Twist()
                self.cmd_vel.publish(self.cmd) 
                # Nothing on ambience
                noises = Ambience()
                self.cmd_lights.publish(noises)

                # Leave error state?
                if not self.is_error():
                    motion_state = 1
                    continue
            elif motion_state == 1:
                # State: 
                #rospy.loginfo("Vehicle Startup In Process")
                # Reset last command setpoint
                self.cmd = Twist()
                self.cmd_vel.publish(self.cmd) 
                # Flash lights and make noises appropriately
                noises = Ambience()
                noises.beacon = Ambience.PATTERN_DFLASH
                noises.headlight = Ambience.PATTERN_DFLASH
                noises.taillight = Ambience.PATTERN_DFLASH
                noises.beep = Ambience.PATTERN_FLASH
                self.cmd_lights.publish(noises)

                # Leave warning state?
                # -> Error
                if self.is_error():
                #    rospy.loginfo("Vehicle Startup Interrupted")
                    motion_state = 0
                    continue
                # -> Motion
                if rospy.get_time() > self.last_error_time + self.motion_delay:
                    rospy.loginfo("Vehicle Startup Complete")
                    motion_state = 2
                    continue
            elif motion_state == 2:
                brakelight = False
                # State:
                # Update setpoints, saturating with acceleration
                # Allow faster absolute deceleration
                # TODO: Unit test the acceleration profile
                # TODO: Move to callback?

                # Translational
                delta = self.cmd_ref.linear.x - self.cmd.linear.x
                if abs(self.cmd_ref.linear.x) > abs(self.cmd.linear.x):
                    # We are increasing our target speed, limit it
                    delta = copysign(min(abs(delta), self.trans_accel*self.period), delta)
                self.cmd.linear.x += delta
                if delta < 0 or self.cmd.linear.x < 0:
                    brakelight = True
                
                # Rotational
                delta = self.cmd_ref.angular.z - self.cmd.angular.z
                if abs(self.cmd_ref.angular.z) > abs(self.cmd.angular.z):
                    # We are increasing our target speed, limit it
                    delta = copysign(min(abs(delta), self.rot_accel*self.period), delta)
                self.cmd.angular.z += delta
                self.cmd_vel.publish(self.cmd)

                # Flash lights and make noises appropriately
                noises = Ambience()
                noises.beacon = Ambience.PATTERN_FLASH
                noises.headlight = Ambience.PATTERN_ON
                if brakelight or self.cmd.linear.x < 0:
                    noises.taillight = Ambience.PATTERN_ON
                else:
                    noises.taillight = Ambience.PATTERN_OFF
                self.cmd_lights.publish(noises)

                # Leave standard state?
                # -> Error
                if self.is_error():
                    motion_state = 0
                    continue
            self.rate.sleep()

    def is_error(self):
        """ Check all error states """
        # Explicit error from board
        if self.control_error:
            self.last_error_time = rospy.get_time()
            rospy.loginfo("Board Control Error")
            return True
        # Haven't received any command for a (short) while
        if rospy.get_time() > self.last_cmd_time + self.no_cmd_timeout:
            self.last_error_time = rospy.get_time()
            rospy.loginfo("Command Timeout Error")
            return True
        # Haven't moved for a while
        if rospy.get_time() > self.last_motion_time + self.no_motion_timeout:
            self.last_error_time = rospy.get_time()
            rospy.loginfo("Motion Timeout Error")
            return True
        return False
   
    def status_callback(self, data):
        """ Receive status. If we have a suitable error, flag """
        # TODO: Use bitfields properly
        if data.error > 1:
            self.control_error = True
        else:
            self.control_error = False
 
    def vel_callback(self, data):
        """ Receive incoming Twist message """
        self.last_cmd_time = rospy.get_time()
        # If the value is non-zero-ish, record that we are seeing
        # valid commands
        # TODO: Do we want to use encoders to check motion instead?
        if abs(data.linear.x) > self.no_motion_value or \
           abs(data.angular.z) > self.no_motion_value:
            self.last_motion_time = rospy.get_time()
        # TODO: Turn this into an object with rest of motion profile?
        # Saturate against speed limits
        x = min(data.linear.x, self.trans_speed)
        x = max(x, -self.trans_speed)
        z = min(data.angular.z, self.rot_speed)
        z = max(z, -self.rot_speed)
        self.cmd_ref = Twist()
        self.cmd_ref.linear.x = x
        self.cmd_ref.angular.z = z

if __name__ == "__main__": 
    MotionManager()
