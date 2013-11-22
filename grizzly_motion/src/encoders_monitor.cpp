
#include "grizzly_motion/encoders_monitor.h"

#include <diagnostic_updater/publisher.h>
#include <grizzly_msgs/Drive.h>
#include <grizzly_msgs/eigen.h>


EncodersMonitor::EncodersMonitor() : encoders_timeout_(0.05)
{
}

/**
 * Separate ROS initialization step for better testability.
 */
void EncodersMonitor::initROS(ros::NodeHandle& nh)
{
  sub_encoders_ = nh.subscribe("motors/encoders", 1, &EncodersMonitor::encodersCallback, this);
  sub_drive_ = nh.subscribe("safe_cmd_drive", 1, &EncodersMonitor::driveCallback, this); 

  double encoders_timeout_seconds;
  ros::param::param<double>("~encoders_timeout", encoders_timeout_seconds, encoders_timeout_.toSec());
  encoders_timeout_ = ros::Duration(encoders_timeout_seconds);
}

template<class M>
static inline ros::Duration age(M msg) 
{
  return ros::Time::now() - msg->header.stamp;
}

bool EncodersMonitor::detectFailedEncoderCandidate()
{
  // Attempt to detect a failed encoder. The symptom will be that the reported velocity will
  // be zero or very near it despite a non-zero commanded velocity. So we compute a vector
  // of differentials between commanded and reported speeds, take the absolute values, and then
  // subtract the mean error from each element. This should expose an outlier scenario where the
  // error in one wheel greatly exceeds that of the other, which could indicate a fault.
  VectorDrive wheelSpeedError = grizzly_msgs::vectorFromDriveMsg(*last_received_encoders_) - 
                                grizzly_msgs::vectorFromDriveMsg(*last_received_drive_);
  wheelSpeedError = wheelSpeedError.cwiseAbs();
  wheelSpeedError -= VectorDrive::Constant(wheelSpeedError.mean());
  
  // Find the index with maximum error, which is our failed encoder candidate.
  VectorDrive::Index maxErrorIndex;
  double max_error = wheelSpeedError.maxCoeff(&maxErrorIndex);

  // If
  return false;
}

bool EncodersMonitor::detectFailedEncoder()
{

}

/**
 * Called in the context of whether cmd_vels may be passed through as safe. Therefore this is 
 * called per-cmd_vel, not upon receipt of encoder messages.
 */
bool EncodersMonitor::ok()
{
  // If we have no encoder data, or its old, then definitely not okay.
  if (!last_received_encoders_ || age(last_received_encoders_) > encoders_timeout_) return false;

  // If we have no drive data, or it's old, then we're just getting rolling, that's fine.
  if (!last_received_drive_ || age(last_received_drive_) > encoders_timeout_) return true;

  if (detectFailedEncoder()) {
    // Not a recoverable fault.
    return false;
  }
 
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
void EncodersMonitor::encodersCallback(const grizzly_msgs::DriveConstPtr& encoders)
{
  last_received_encoders_ = encoders;
}

/**
 * New commands went out to the motors.
 */
void EncodersMonitor::driveCallback(const grizzly_msgs::DriveConstPtr& drive)
{
  last_received_drive_ = drive;
}

