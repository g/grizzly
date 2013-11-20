
#include "grizzly_motion/encoders_monitor.h"

#include <diagnostic_updater/publisher.h>
#include <grizzly_msgs/Drive.h>

EncodersMonitor::EncodersMonitor() : nh_("")
{
  sub_encoders_ = nh_.subscribe("encoders", 1, &EncodersMonitor::encoders_callback, this);
  sub_drive_ = nh_.subscribe("safe_cmd_drive", 1, &EncodersMonitor::drive_callback, this); 

  double encoders_timeout_seconds;
  ros::param::param<double>("~encoders_timeout", encoders_timeout_seconds, 0.05);
  encoders_timeout_ = ros::Duration(encoders_timeout_seconds);
}

template<class M>
static inline ros::Duration age(M msg) 
{
  return ros::Time::now() - msg->header.stamp;
}

/**
 * Called in the context of whether cmd_vels may be passed through as safe.
 */
bool EncodersMonitor::ok()
{
  // If we have no encoder data, or its old, then definitely not okay.
  if (!last_received_encoders_) return false;
  if (age(last_received_encoders_) > encoders_timeout_) return false;

  return true;
}

/**
 * Prepare diagnostics. Called at 1Hz by the Updater.
 */
void EncodersMonitor::diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!last_received_encoders_)
  {
    stat.summary(2, "No encoders messages received.");
    return;
  } 
  if (age(last_received_encoders_) > encoders_timeout_)
  {
    stat.summaryf(2, "Last encoders message is stale (%f seconds old). Check motor driver connectivity.",
        age(last_received_encoders_).toSec());
    return;
  }

  stat.summary(1, "Encoders monitoring not implemented.");
  //stat.summary(0, "Encoders look good.")
}

/**
 * New encoder data received. The important logic here is detecting when an encoder
 * or the associated cabling has failed. The general principle is:
 *   - compared commanded speed to actual speed (for an "error" value)
 *   - monitor how much time each encoder spends over an acceptable threshold of error.
 *   - compute the variance of the time-in-excess-error array. If it exceeds a
 *     a heuristically-set value, conclude that there may be a failed encoder.
 * Future work on this function could also account for current consumption by motors;
 * eg, a spike in current by a motor relative to the others with no associated speed change.
 */
void EncodersMonitor::encoders_callback(const grizzly_msgs::DriveConstPtr& encoders)
{
  last_received_encoders_ = encoders;
}

/**
 * New commands went out to the motors.
 */
void EncodersMonitor::drive_callback(const grizzly_msgs::DriveConstPtr& drive)
{
  last_received_drive_ = drive;
}

