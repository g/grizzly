/**
Software License Agreement (BSD)

\file      dead_reckoning_node.h
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

#include "nav_msgs/Odometry.h"
#include "grizzly_msgs/Drive.h"
#include "sensor_msgs/JointState.h"

#include <Eigen/Core>
#include <Eigen/Dense>

const Eigen::MatrixXd ODOM_POSE_COVAR_MOTION = (Eigen::MatrixXd(6, 6) << 
  1e-3, 0,    0,   0,   0,   0, 
  0,    1e-1, 0,   0,   0,   0,
  0,    0,    1e6, 0,   0,   0,
  0,    0,    0,   1e6, 0,   0,
  0,    0,    0,   0,   1e6, 0,
  0,    0,    0,   0,   0,   0.174).finished();

const Eigen::MatrixXd ODOM_POSE_COVAR_NOMOVE = (Eigen::MatrixXd(6, 6) << 
  1e-9, 0,    0,   0,   0,   0, 
  0,    1e-9, 0,   0,   0,   0,
  0,    0,    1e6, 0,   0,   0,
  0,    0,    0,   1e6, 0,   0,
  0,    0,    0,   0,   1e6, 0,
  0,    0,    0,   0,   0,   1e-9).finished();

const Eigen::MatrixXd ODOM_TWIST_COVAR_MOTION = (Eigen::MatrixXd(6, 6) << 
  1e-3, 0,    0,   0,   0,   0, 
  0,    1e-1, 0,   0,   0,   0,
  0,    0,    1e6, 0,   0,   0,
  0,    0,    0,   1e6, 0,   0,
  0,    0,    0,   0,   1e6, 0,
  0,    0,    0,   0,   0,   0.174).finished();

const Eigen::MatrixXd ODOM_TWIST_COVAR_NOMOVE = (Eigen::MatrixXd(6, 6) << 
  1e-9, 0,    0,   0,   0,   0, 
  0,    1e-9, 0,   0,   0,   0,
  0,    0,    1e6, 0,   0,   0,
  0,    0,    0,   1e6, 0,   0,
  0,    0,    0,   0,   1e6, 0,
  0,    0,    0,   0,   0,   1e-9).finished();


class DeadReckoning
{
public:
  DeadReckoning(double vehicle_width, double wheel_radius) 
  : max_dt_(0.1), width_(vehicle_width), radius_(wheel_radius), initialize(false)
  {
  }

  bool next(const grizzly_msgs::DriveConstPtr& encoders, nav_msgs::Odometry* odom, sensor_msgs::JointState* joints);
 
protected:
  // Maintaining state between encoder updates.
  ros::Time last_time_;
  Eigen::Vector2f last_vels_;

  geometry_msgs::Point position_; 
  geometry_msgs::Twist twist_; 
  double yaw_;
  bool initialize;
  std::vector<std::string> joint_name_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> last_joint_pos_;

  // Configuration
  double width_, radius_;
  ros::Duration max_dt_;
};

