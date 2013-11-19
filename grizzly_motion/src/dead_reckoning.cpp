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

/**
 * Open-loop mapping between linear/angular commands and individual wheel speed
 * commands. Currently very naive, but in the future may provide some further
 * intelligence, though not closed-loop control.
 */
void DeadReckoning::encodersCallback(const grizzly_msgs::DriveConstPtr& encoders)
{
  nav_msgs::Odometry odom;
  if (nextEncoders(encoders, &odom)) {
    pub_.publish(odom);
  }
}

bool DeadReckoning::nextEncoders(const grizzly_msgs::DriveConstPtr& encoders, nav_msgs::Odometry* odom)
{
  bool success = false;

  // Angular velocity per-side in rad/s, velocity per-size in m/s
  Vector2f avg_rotations((encoders->front_left + encoders->rear_left) / 2,
                         (encoders->front_right + encoders->rear_right) / 2);
  Vector2f vels = avg_rotations * radius_;

  ros::Duration dt = encoders->header.stamp - last_time_;
  if (dt <= max_dt_) {
    // Integrate position based on previous yaw and speed
     position_.x += cos(yaw_) * twist_.linear.x * dt.toSec();
     position_.y += sin(yaw_) * twist_.linear.y * dt.toSec();

    // Update heading by integrating previous angular velocity.
     yaw_ += twist_.angular.z * dt.toSec();

    // Update linear and angular velocity
    twist_.linear.x = vels.mean();
    twist_.angular.z = (vels[1] - vels[0]) / width_;
    
    // Timestamp from encoder message, set frames correctly.
    odom->header = encoders->header;
    odom->header.frame_id = "odom_combined";
    odom->child_frame_id = "base_footprint"; 
    odom->pose.pose.position = position_;
    odom->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_);
    odom->twist.twist = twist_;
    success = true;
  } else {
    static bool first = true;
    ROS_WARN_COND(first, "Gap between encoders messages is too large, no odom generated.");
    first = false;
  }

  last_time_ = encoders->header.stamp;
  last_vels_ = vels;
  return success;
}
