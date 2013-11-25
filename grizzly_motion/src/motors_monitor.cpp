
#include "grizzly_motion/motors_monitor.h"
#include <diagnostic_updater/publisher.h>
#include <roboteq_msgs/Feedback.h>
#include <roboteq_msgs/Status.h>
#include <grizzly_msgs/eigen.h>
#include <boost/bind.hpp>


MotorsMonitor::MotorsMonitor() : nh_("")
{

  double motors_timeout_seconds;
  ros::param::param<double>("~motors_timeout", motors_timeout_seconds, 0.1);
  motors_timeout_ = ros::Duration(motors_timeout_seconds);

  fb_sub_[0] = nh_.subscribe<roboteq_msgs::Feedback>("/motors/front_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::FrontLeft));
  fb_sub_[1] = nh_.subscribe<roboteq_msgs::Feedback>("/motors/front_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::FrontRight));
  fb_sub_[2] = nh_.subscribe<roboteq_msgs::Feedback>("/motors/rear_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::RearLeft));
  fb_sub_[3] = nh_.subscribe<roboteq_msgs::Feedback>("/motors/rear_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::RearRight));

  stat_sub_[0] = nh_.subscribe<roboteq_msgs::Status>("/motors/front_right/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::FrontLeft));
  stat_sub_[1] = nh_.subscribe<roboteq_msgs::Status>("/motors/front_left/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::FrontRight));
  stat_sub_[2] = nh_.subscribe<roboteq_msgs::Status>("/motors/rear_right/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::RearLeft));
  stat_sub_[3] = nh_.subscribe<roboteq_msgs::Status>("/motors/rear_left/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::RearRight));
}

template<class M>
static inline ros::Duration age(M msg) 
{
  return ros::Time::now() - msg->header.stamp;
}

/**
 * Called in the context of whether cmd_vels may be passed through as safe.
 */
bool MotorsMonitor::ok()
{
  // If we have no data, or its old, then definitely not okay.
  for (int i=0;i<4;i++) {
    if (!last_received_status_[i] || !last_received_feedback_) return false;
    if (age(last_received_status_[i]) > motors_timeout_) return false;
  }

  return true;
}

/**
 * Prepare diagnostics. Called at 1Hz by the Updater.
 */
void MotorsMonitor::diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  for (int i=grizzly_msgs::Drives::FrontLeft;i<=grizzly_msgs::Drives::RearRight;i++) {
    if (!last_received_status_[i] || !last_received_feedback_[i])
    {
      stat.summary(2, "Motor " + grizzly_msgs::nameFromDriveIndex(i) + " driver status and/or feedback message not received");
      return;
    } 
    if (age(last_received_status_[i]) > motors_timeout_)
    {
      stat.summaryf(2, "Last motor %s status message is stale (%f seconds old). Check motor driver connectivity.",grizzly_msgs::nameFromDriveIndex(i).c_str(),age(last_received_status_[i]).toSec());
      return;
    }
  }

  int fault_level = 0;
  std::string fault_output = "";

  for (int i=grizzly_msgs::Drives::FrontLeft;i<=grizzly_msgs::Drives::RearRight;i++) {
    if(last_received_status_[i]->status != 0) {
      fault_level = lookForSeriousFault(last_received_status_[i]->status,stat,i);
      if (fault_level == 1) 
        fault_output = "Motor at " + grizzly_msgs::nameFromDriveIndex(i) + " is in a fault state";
      else if (fault_level == 2) 
        fault_output = "Motor at " + grizzly_msgs::nameFromDriveIndex(i) + " is in a serious fault state";
      else
        fault_output = "Motor at " + grizzly_msgs::nameFromDriveIndex(i) + " is cleared of faults";

      stat.summary(fault_level,fault_output);
      return;
    }
  } 

}

int MotorsMonitor::lookForSeriousFault(uint8_t status,diagnostic_updater::DiagnosticStatusWrapper& stat, const int motor_num)
{
  int error_level = 0;
  roboteq_msgs::Status fault_def;
 
  //Warning level faults 
  if (status & fault_def.FAULT_UNDERVOLTAGE) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "Undervoltage fault");
    error_level = 1;
  }

  if (status & fault_def.FAULT_EMERGENCY_STOP) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "Emergency Stop fault");
    error_level = 1;
  }

  if (status & fault_def.FAULT_SEPEX_EXCITATION_FAULT) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "Sepex Excitation fault");
    error_level = 1;
  }

  if (status & fault_def.FAULT_STARTUP_CONFIG_FAULT) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "Startup Configuration fault");
    error_level = 1;
  }

  //More serious faults 
  if (status & fault_def.FAULT_OVERHEAT) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "Overheat fault");
    error_level = 2;
  }

  if (status & fault_def.FAULT_OVERVOLTAGE) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "Overvoltage fault");
    error_level = 2;
  }

  if (status & fault_def.FAULT_SHORT_CIRCUIT) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "Short Circuit fault");
    error_level = 2;
  } 

  if (status & fault_def.FAULT_MOSFET_FAILURE) {
    stat.add (grizzly_msgs::nameFromDriveIndex(motor_num), "MOSFET Failure fault");
    error_level = 2;
  }

  return error_level;

}

void MotorsMonitor::motor_feedback(const roboteq_msgs::FeedbackConstPtr msg, const int motor_num) {
  last_received_feedback_[motor_num] = msg;
}

void MotorsMonitor::motor_status(const roboteq_msgs::StatusConstPtr& msg, const int motor_num) {
  last_received_status_[motor_num] = msg;
}

