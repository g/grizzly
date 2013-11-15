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

#include "eigen_conversions/eigen_msg.h"
//#include "geometry_msgs/Quaternion.h"
//#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "grizzly_msgs/Drive.h"

#include <Eigen/Core>

using Eigen::Vector2f;

/**
 */
class DeadReckoning
{
public:
  DeadReckoning() : nh_(""), max_dt_(0.2)
  {
    ros::param::get("vehicle_width", width_);
    ros::param::get("wheel_radius", radius_);

    pub_ = nh_.advertise<nav_msgs::Odometry>("odom_encoder", 1);
    sub_ = nh_.subscribe("encoders", 1, &DeadReckoning::encodersCallback, this);
  }

protected:
  void encodersCallback(const grizzly_msgs::DriveConstPtr&);

  // ROS interface
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  // Maintaining state between encoder updates.
  ros::Time last_time_;
  Vector2f last_vels_;
  /*geometry_msgs::Point position_; 
  geometry_msgs::Twist twist_; 
  double yaw_;*/
  Eigen::Affine3d pose_;
  Eigen::Matrix<double,6,1> twist_;

  // Configuration
  double width_, radius_;
  ros::Duration max_dt_;
};

/**
 * Open-loop mapping between linear/angular commands and individual wheel speed
 * commands. Currently very naive, but in the future may provide some further
 * intelligence, though not closed-loop control.
 */
void DeadReckoning::encodersCallback(const grizzly_msgs::DriveConstPtr& encoders)
{
  // Angular velocity per-side in rad/s, velocity per-size in m/s
  Vector2f avg_rotations((encoders->front_left + encoders->rear_left) / 2,
                         (encoders->front_right + encoders->rear_right) / 2);
  Vector2f vels = avg_rotations * radius_;

  ros::Duration dt = encoders->header.stamp - last_time_;
  if (dt <= max_dt_) {
    // Integrate position based on previous yaw and speed
    // position_.x += cos(yaw_) * twist_.linear.x * dt.toSec();
    // position_.y += sin(yaw_) * twist_.linear.y * dt.toSec();
    //pose_ = pose_.translate();

    // Update heading by integrating previous angular velocity.
    // yaw_ += twist_.angular.z * dt.toSec();
    // Matrix3f m = AngleAxisf(yaw_, Vector3f::UnitZ());

    // Update linear and angular velocity
    //twist_.linear.x = vels.mean();
    //twist_.angular.z = (vels[1] - vels[0]) / width_;
    
    // Timestamp from encoder message, set frames correctly.
    nav_msgs::Odometry odom;
    odom.header = encoders->header;
    odom.header.frame_id = "odom_combined";
    odom.child_frame_id = "base_footprint"; 
    tf::twistEigenToMsg(twist_, odom.twist.twist);
    tf::poseEigenToMsg(pose_, odom.pose.pose);
    pub_.publish(odom);
  } else {
    static bool first = true;
    ROS_WARN_COND(first, "Gap between encoders messages is too large, not publishing odom.");
    first = false;
  }

  last_time_ = encoders->header.stamp;
  last_vels_ = vels;
}

/**
 * Main entry point.
 */
int main (int argc, char ** argv)
{
  ros::init(argc, argv, "grizzly_dead_reckoning"); 
  DeadReckoning dr;
  ros::spin();
}
