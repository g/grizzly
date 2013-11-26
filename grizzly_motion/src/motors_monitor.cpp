
#include "grizzly_motion/motors_monitor.h"
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

  for (int i=grizzly_msgs::Drives::FrontLeft;i<=grizzly_msgs::Drives::RearRight;i++) {
    fault_level_[i] = 0;
  } 
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
  for (int i=grizzly_msgs::Drives::FrontLeft;i<=grizzly_msgs::Drives::RearRight;i++) {
    if (!last_received_status_[i] || !last_received_feedback_) return false;
    if (age(last_received_status_[i]) > motors_timeout_) return false;
    fault_level_[i] = lookForSeriousFault (last_received_status_[i]->status, i);
    if (fault_level_[i] > 0)
      return false;
  }

  return true;
}

int MotorsMonitor::lookForSeriousFault(uint8_t status, const int motor_num)
{
  int error_level = 0;
  roboteq_msgs::Status fault_def;
 
  //Warning level faults 
  if (status & fault_def.FAULT_UNDERVOLTAGE) {
    error_level = 1;
  }

  if (status & fault_def.FAULT_EMERGENCY_STOP) {
    error_level = 1;
  }

  if (status & fault_def.FAULT_SEPEX_EXCITATION_FAULT) {
    error_level = 1;
  }

  if (status & fault_def.FAULT_STARTUP_CONFIG_FAULT) {
    error_level = 1;
  }

  //More serious faults 
  if (status & fault_def.FAULT_OVERHEAT) {
    error_level = 2;
  }

  if (status & fault_def.FAULT_OVERVOLTAGE) {
    error_level = 2;
  }

  if (status & fault_def.FAULT_SHORT_CIRCUIT) {
    error_level = 2;
  } 

  if (status & fault_def.FAULT_MOSFET_FAILURE) {
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

