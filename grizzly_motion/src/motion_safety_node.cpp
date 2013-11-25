/**
Software License Agreement (BSD)

\file      motion_safety_node.cpp
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

#include "grizzly_motion/motion_safety.h"
#include "grizzly_motion/encoders_monitor.h"
#include "grizzly_motion/change_limiter.h"

#include "grizzly_msgs/Ambience.h"
#include "grizzly_msgs/Drive.h"
#include "grizzly_msgs/RawStatus.h"
#include "std_msgs/Bool.h"

#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"

using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::FrequencyStatusParam;
using diagnostic_updater::HeaderlessTopicDiagnostic;
using diagnostic_updater::Updater;

MotionSafety::MotionSafety(ros::NodeHandle* nh)
  : nh_(nh)
{
  ros::param::get("vehicle_width", width_);
  ros::param::get("wheel_radius", radius_);
  ros::param::get("max_acceleration", max_accel_);  // m/s^2
  starting_duration_ = ros::Duration(2.0);

  // Drive pass-through
  sub_drive_ = nh_->subscribe("cmd_drive", 1, &MotionSafety::driveCallback, this);
  pub_safe_drive_ = nh_->advertise<grizzly_msgs::Drive>("safe_cmd_drive", 1);

  // MCU interface
  pub_ambience_ = nh_->advertise<grizzly_msgs::Ambience>("mcu/ambience", 1);
  pub_estop_ = nh_->advertise<std_msgs::Bool>("mcu/estop", 1);
  sub_mcu_status_ = nh_->subscribe("mcu/status", 1, &MotionSafety::mcuStatusCallback, this);
  watchdog_timer_ = nh_->createTimer(ros::Duration(0.05), &MotionSafety::watchdogCallback, this);

  // Set up the diagnostic updater
  diagnostic_updater_.reset(new Updater());
  diagnostic_updater_->setHardwareID("grizzly");

  // Frequency report on statuses coming from the MCU.
  expected_mcu_status_frequency_ = 10;
  diag_mcu_status_freq_.reset(new HeaderlessTopicDiagnostic("MCU status", *diagnostic_updater_,
      FrequencyStatusParam(&expected_mcu_status_frequency_, &expected_mcu_status_frequency_, 0.01, 5.0)));

  // Frequency report on inbound drive messages.
  min_cmd_drive_freq_ = 10;
  max_cmd_drive_freq_ = 50;
  diag_cmd_drive_freq_.reset(new HeaderlessTopicDiagnostic("Drive command", *diagnostic_updater_, 
      FrequencyStatusParam(&min_cmd_drive_freq_, &max_cmd_drive_freq_, 0.01, 5.0)));

  // More specialized monitoring for encoders. 
  encoders_monitor_.reset(new EncodersMonitor(nh_));
  diagnostic_updater_->add("Encoders", encoders_monitor_.get(), &EncodersMonitor::diagnostic);

  // Rate-of-change limiter for wheel speed commands.
  accel_limiters_[0].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::front_left));
  accel_limiters_[1].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::front_right));
  accel_limiters_[2].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::rear_left));
  accel_limiters_[3].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::rear_right));
}

void MotionSafety::watchdogCallback(const ros::TimerEvent&)
{
  // Messages to be published to the MCU at each run of this function.
  grizzly_msgs::Ambience ambience;
  std_msgs::Bool estop;
  estop.data = false;

  bool ok = encoders_monitor_->ok();  // && motor_drivers_monitor->ok(); 

  if (state_ == MotionStates::Stopped)
  {
    if (ros::Time::now() - last_commanded_movement_time_ < ros::Duration(0.1))
    {
      state_ = MotionStates::Starting;
      transition_to_moving_time_ = ros::Time::now() + starting_duration_;
    }
  }

  if (state_ == MotionStates::Starting)
  {
    ambience.beacon = ambience.headlight = ambience.taillight = ambience.beep =
      grizzly_msgs::Ambience::PATTERN_DFLASH;
    if (ros::Time::now() > transition_to_moving_time_) state_ = MotionStates::Moving;
    if (!ok) state_ = MotionStates::PendingStopped;
  }

  if (state_ == MotionStates::Moving)
  {
    if (!ok) state_ = MotionStates::PendingStopped;
  }

  if (state_ == MotionStates::PendingStopped)
  {
    estop.data = true;
    if (!encoders_monitor_->moving()) state_ = MotionStates::Stopped;
  }
  
  if (state_ == MotionStates::Fault)
  {
    estop.data = true;
  }

  diagnostic_updater_->update(); 
  pub_ambience_.publish(ambience);
  pub_estop_.publish(estop);
}

/**
 * Manages a pass-through of Grizzly Drive messages, ensuring that the appropriate
 * delays are observed before allowing the chassis to move, including activating
 * the chassis lights and beeper. Also monitors encoders for possible failure.
 */
void MotionSafety::driveCallback(const grizzly_msgs::DriveConstPtr& drive_commanded)
{
  diag_cmd_drive_freq_->tick();

  // This signals the main loop function to begin a transition to the Moving state.
  if (!grizzly_msgs::isStationary(*drive_commanded.get()))
    last_commanded_movement_time_ = drive_commanded->header.stamp;
 
  grizzly_msgs::Drive drive_safe;
  drive_safe.header = drive_commanded->header;

  if (state_ == MotionStates::Moving)
  {
    for (int wheel = 0; wheel < 4; wheel++) 
      accel_limiters_[wheel]->apply(drive_commanded.get(), &drive_safe);
  }

  pub_safe_drive_.publish(drive_safe);
}

void MotionSafety::mcuStatusCallback(const grizzly_msgs::RawStatus& status)
{
  diag_mcu_status_freq_->tick();
}

/**
 * Node entry point.
 */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "grizzly_motion_safety"); 
  ros::NodeHandle nh("");
  MotionSafety ms(&nh);
  ros::spin();
}

