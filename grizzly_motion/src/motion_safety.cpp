/**
Software License Agreement (BSD)

\file      motion_safety.cpp
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

#include <ros/ros.h>
#include <grizzly_msgs/Ambience.h>
#include <grizzly_msgs/Drive.h>
#include <grizzly_msgs/RawStatus.h>

/**
 * Manages a pass-through of Grizzly Drive messages, ensuring that the appropriate
 * delays are observed before allowing the chassis to move, including activating
 * the chassis lights and beeper. Also monitors encoders for possible failure.
 */
class MotionSafety
{
public:
  MotionSafety() : nh_("")
  {
    //ros::param::param<double>("~vehicle_width", width_, 1.01);
    //ros::param::param<double>("~wheel_radius", radius_, 0.333);

    pub_safe_drive_ = nh_.advertise<grizzly_msgs::Drive>("safe_cmd_drive", 1);
    pub_ambience_ = nh_.advertise<grizzly_msgs::Ambience>("mcu/ambience", 1);

    sub_drive_ = nh_.subscribe("cmd_drive", 1, &MotionSafety::drive_callback, this);
    sub_encoders_ = nh_.subscribe("encoders", 1, &MotionSafety::encoders_callback, this);
    sub_mcu_status_ = nh_.subscribe("mcu/status", 1, &MotionSafety::mcu_status_callback, this);
  }

protected:
  void drive_callback(const grizzly_msgs::DriveConstPtr&);
  void encoders_callback(const grizzly_msgs::DriveConstPtr&);
  void mcu_status_callback(const grizzly_msgs::RawStatus&);
  
  ros::NodeHandle nh_;
  ros::Publisher pub_safe_drive_, pub_ambience_;
  ros::Subscriber sub_drive_, sub_encoders_, sub_mcu_status_;
};

void MotionSafety::drive_callback(const grizzly_msgs::DriveConstPtr& drive_commanded)
{
  pub_safe_drive_.publish(drive_commanded);
}

void MotionSafety::encoders_callback(const grizzly_msgs::DriveConstPtr& encoders)
{
}

void MotionSafety::mcu_status_callback(const grizzly_msgs::RawStatus& status)
{
}

/**
 * Node entry point.
 */
int main (int argc, char ** argv)
{
  ros::init(argc, argv, "grizzly_motion_safety"); 
  MotionSafety ms;
  ros::spin();
}
