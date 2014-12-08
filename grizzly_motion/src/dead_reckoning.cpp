/**
Software License Agreement (BSD)

\file      dead_reckoning.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "grizzly_motion/dead_reckoning.h"

using Eigen::Vector2f;

/**
 * Open-loop mapping between linear/angular commands and individual wheel speed
 * commands. Currently very naive, but in the future may provide some further
 * intelligence, though not closed-loop control.
 */
void wrap_rad(double& r){ r = fabs(r) < M_PI ? r : (r -= copysignf(1.0, r) * 2.0 * M_PI);}

bool DeadReckoning::next(const grizzly_msgs::DriveConstPtr& encoders, nav_msgs::Odometry* odom, sensor_msgs::JointState* joints)
{
  bool success = false;

  if(!initialize) { 
    joint_name_.push_back("joint_back_left_wheel");
    joint_name_.push_back("joint_back_right_wheel");
    joint_name_.push_back("joint_front_left_wheel");
    joint_name_.push_back("joint_front_right_wheel");
    joint_pos_.resize(joint_name_.size(), 0.0);
    joint_vel_.resize(joint_name_.size(), 0.0);
    last_joint_pos_.resize(joint_name_.size(), 0.0); 
    last_time_ = encoders->header.stamp; 
    yaw_ = 0.0;  
    initialize = true;

    ROS_INFO("initialization complete.");
  }

  // Angular velocity per-side in rad/s, velocity per-size in m/s
  Vector2f avg_rotations((encoders->front_left + encoders->rear_left) / 2,
                         (encoders->front_right + encoders->rear_right) / 2);
  Vector2f vels = avg_rotations * radius_;

  ros::Duration dt = encoders->header.stamp - last_time_;
  if (dt <= max_dt_) {
    // Integrate position based on previous yaw and speed
     position_.x += cos(yaw_) * twist_.linear.x * dt.toSec();
     position_.y += sin(yaw_) * twist_.linear.x * dt.toSec();

    // Update heading by integrating previous angular velocity.
     yaw_ += twist_.angular.z * dt.toSec();

    // Update linear and angular velocity
    twist_.linear.x = vels.mean();
    twist_.angular.z = (vels[1] - vels[0]) / width_;

    //update joint's velocity
    joint_vel_[0] = encoders->rear_left;
    joint_vel_[1] = encoders->rear_right;
    joint_vel_[2] = encoders->front_left;
    joint_vel_[3] = encoders->front_right;

    // Timestamp from encoder message, set frames correctly.
    odom->header = encoders->header;
    odom->header.frame_id = "odom";
    odom->child_frame_id = "base_footprint"; 
    odom->pose.pose.position = position_;
    odom->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_);
    odom->twist.twist = twist_;

    Eigen::Map<Eigen::MatrixXd> poseCov(odom->pose.covariance.data(), 6, 6);
    Eigen::Map<Eigen::MatrixXd> twistCov(odom->twist.covariance.data(), 6, 6);

    if(fabs(twist_.linear.x) <= 1e-3 && fabs(twist_.angular.z) <= 1e-3) {
      poseCov = ODOM_POSE_COVAR_NOMOVE;
      twistCov = ODOM_TWIST_COVAR_NOMOVE;
      joint_pos_ = last_joint_pos_;
    } 
    else {
      poseCov = ODOM_POSE_COVAR_MOTION;
      twistCov = ODOM_TWIST_COVAR_MOTION;

      // update joint's position
      for(uint8_t i = 0; i < joint_name_.size(); i++) {
        joint_pos_[i] = last_joint_pos_[i] + dt.toSec() * joint_vel_[i]; 
        wrap_rad(joint_pos_[i]);
      }
    }
    joints->name = joint_name_;
    joints->velocity = joint_vel_;
    joints->position = joint_pos_;
    joints->header = encoders->header;
    success = true;
  } else {
    static bool first = true;
    ROS_WARN_COND(first, "Gap between encoders messages is too large, no odom generated.");
    first = false;
  }

  last_time_ = encoders->header.stamp;
  last_joint_pos_ = joint_pos_;
  last_vels_ = vels;
  return success;
}
