#!/usr/bin/env python
import roslib; roslib.load_manifest('grizzly_node')
import roslib.rosenv
import rospy
import tf

from math import sin,cos,pi
from geometry_msgs.msg import Quaternion
from roboteq_msgs.msg import Feedback
from nav_msgs.msg import Odometry

ODOM_POSE_COVAR_MOTION = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e6]
ODOM_POSE_COVAR_NOMOVE = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVAR_MOTION = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e6]
ODOM_TWIST_COVAR_NOMOVE = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]
FR = 0
FL = 1
RR = 2
RL = 3

class DeadReckoning(object):
    def __init__(self):
        rospy.init_node('dead_reckoning')
        
        self.latest_enc_reading = [0,0,0,0]
        self.new_data = [True, True, True, True]
        self.last_time = rospy.get_time()
        self.last_l_w_vel = 0
        self.last_r_w_vel = 0

        # Parameters
        self.width = rospy.get_param('~width',1.01)
        self.gear_down = rospy.get_param('~gearing', 50.0)
        self.wheel_radius = rospy.get_param('~wheel_radius',0.333)

        # Initialize odometry message
        self.odom = Odometry()
        self.odom.header.frame_id = "odom_combined"
        self.odom.child_frame_id = "base_footprint" 
        self.last_encoder = []


        # Publishers and Subscribers
        self.pub_enc_odom = rospy.Publisher('encoder',Odometry);
        rospy.Subscriber('motors/front_right/feedback', Feedback, self.HandleFREnc)
        rospy.Subscriber('motors/front_left/feedback', Feedback, self.HandleFLEnc)
        rospy.Subscriber('motors/rear_right/feedback', Feedback, self.HandleRREnc)
        rospy.Subscriber('motors/rear_left/feedback', Feedback, self.HandleRLEnc)




    #Callbacks gear down RPM data and report new data
    def HandleFREnc(self, data):
        self.latest_enc_reading[FR] = data.encoder_rpm[0]/self.gear_down
        self.new_data[FR] = True
        self.process_encoders()

    def HandleFLEnc(self, data):
        self.latest_enc_reading[FL] = data.encoder_rpm[0]/self.gear_down
        self.new_data[FL] = True
        self.process_encoders()

    def HandleRREnc(self, data):
        self.latest_enc_reading[RR] = data.encoder_rpm[0]/self.gear_down
        self.new_data[RR] = True
        self.process_encoders()

    def HandleRLEnc(self, data):
        self.latest_enc_reading[RL] = data.encoder_rpm[0]/self.gear_down
        self.new_data[RL] = True
        self.process_encoders()

    def process_encoders(self):
        if (not (False in self.new_data)): #received new data from all four motorsi
            #reset new data flags
            self.new_data = [False, False, False, False]

            #average left and right RPM readings
            leftRPM = (self.latest_enc_reading[FL] + self.latest_enc_reading[RL])/2.0  
            rightRPM = -(self.latest_enc_reading[FR] + self.latest_enc_reading[RR])/2.0 # change from motor frame to body frame

            #convert to rotations/s
            curr_l_rots = leftRPM/(60) 
            curr_r_rots = rightRPM/(60)

            #convert rotations/s to m/s
            l_w_vel = curr_l_rots * 2 * pi * self.wheel_radius
            r_w_vel= curr_r_rots * 2 * pi * self.wheel_radius

            #trapezoidal integration to get distance travelled since last velocity measurement
            curr_time = rospy.get_time()
            dt = curr_time - self.last_time
            left_travel = dt * (l_w_vel + self.last_l_w_vel)/2.0
            right_travel = dt * (r_w_vel + self.last_r_w_vel)/2.0
            self.last_time = curr_time

            #save wheel velocities
            self.last_l_w_vel = l_w_vel
            self.last_r_w_vel = r_w_vel

            #calculate body translational and rotational travel
            dr = (left_travel + right_travel) / 2
            da = (right_travel - left_travel) / self.width

            #calculate body translational and rotational velocities (ENU)
            ddr = (l_w_vel + r_w_vel) / 2
            dda = (r_w_vel - l_w_vel) / self.width
            
            #calculate odometry
            o = self.odom.pose.pose.orientation
            (r, p, curr_heading) = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
            self.odom.pose.pose.position.x += dr * cos(curr_heading)
            self.odom.pose.pose.position.y += dr * sin(curr_heading) 
            quat = tf.transformations.quaternion_from_euler(0,0,curr_heading + da)
            self.odom.pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

            self.odom.twist.twist.linear.x = ddr
            self.odom.twist.twist.angular.z = dda
            self.odom.header.stamp = rospy.Time.now()

            #enter covariances and publish data
            if leftRPM == 0 and rightRPM == 0:
                self.odom.pose.covariance = ODOM_POSE_COVAR_NOMOVE
                self.odom.twist.covariance = ODOM_TWIST_COVAR_NOMOVE
            else:
                self.odom.pose.covariance = ODOM_POSE_COVAR_MOTION
                self.odom.twist.covariance = ODOM_TWIST_COVAR_MOTION

            self.pub_enc_odom.publish(self.odom)

    
if __name__ == "__main__":
    obj = DeadReckoning()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
