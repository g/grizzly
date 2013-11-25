
#include "grizzly_motion/motors_monitor.h"
#include <diagnostic_updater/publisher.h>
#include <roboteq_msgs/Feedback.h>
#include <roboteq_msgs/Status.h>
#include <grizzly_msgs/eigen.h>
#include <boost/bind.hpp>


MotorsMonitor::MotorsMonitor() : nh_("")
{

/*  sub_encoders_ = nh_.subscribe("motors/encoders", 1, &EncodersMonitor::encoders_callback, this);
  sub_drive_ = nh_.subscribe("safe_cmd_drive", 1, &EncodersMonitor::drive_callback, this); */

  double motors_timeout_seconds;
  ros::param::param<double>("~motors_timeout", motors_timeout_seconds, 0.1);
  motors_timeout_ = ros::Duration(motors_timeout_seconds);
  fb_sub_[0] = nh_.subscribe<roboteq_msgs::Feedback>("/motors/front_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::FrontRight));
//  fb_sub_[1] = nh_.subscribe<grizzly_msgs::Feedback>("/motors/front_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::FrontLeft));
//  fb_sub_[2] = nh_.subscribe<grizzly_msgs::Feedback>("/motors/rear_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::RearRight));
//  fb_sub_[3] = nh_.subscribe<grizzly_msgs::Feedback>("/motors/rear_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::RearLeft));

  stat_sub_[0] = nh_.subscribe<roboteq_msgs::Feedback>("/motors/front_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::FrontRight));
//  stat_sub_[1] = nh_.subscribe<grizzly_msgs::Feedback>("/motors/front_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::FrontLeft));
//  stat_sub_[2] = nh_.subscribe<grizzly_msgs::Feedback>("/motors/rear_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::RearRight));
//  stat_sub_[3] = nh_.subscribe<grizzly_msgs::Feedback>("/motors/rear_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,grizzly_msgs::Drives::RearLeft));
  
  clean_status_ = true; //assume status is fine 


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
  for (int i=0;i<4;i++) {
    if (!last_received_status_[i] || !last_received_feedback_[i])
    {
      stat.summary(2, "Motor driver status and/or feedback message not received");
      return;
    } 
    if (age(last_received_status_[i]) > motors_timeout_)
    {
      stat.summary(2, "Last motor status message is stale (%f seconds old). Check motor driver connectivity.",
          age(last_received_status_).toSec());
      return;
    }
  }

  int fault_level = 0;
  std::string fault_string = "";
  std::string fault_output = "No Motor Faults";

  for (int i=0;i<4;i++) {
    if(last_received_status_[i]->status != 0) {
      fault_level = lookForSeriousFault(last_received_status_[i]->status, fault_string);
      fault_output = grizzly_msgs::nameFromDriveIndex(i);
      fault_output = fault_output + "has" + fault_string + " faults";
      stat.summary(fault_level, fault_output.c_str());
      return;
    }
  } 

}

//TODO: Definitely need a better way of doing this. May need to change roboteq_msgs::Status to include strings
int MotorsMonitor::lookForSeriousFault(uint8_t status, std::string& faults)
{
  int error_level = 0;
  roboteq_msgs::Status fault_def;
 
  //Warning level faults 
  if (status & fault_def.FAULT_UNDERVOLTAGE) {
    faults = faults + std::string("undervoltage,");
    error_level = 1;
  }

  if (status & fault_def.FAULT_EMERGENCY_STOP) {
    faults = faults + std::string("emergency stop,");
    error_level = 1;
  }

  if (status & fault_def.FAULT_SEPEX_EXCITATION_FAULT) {
    faults = faults + std::string("sepex excitation fault,");
    error_level = 1;
  }

  if (status & fault_def.FAULT_STARTUP_CONFIG_FAULT) {
    faults = faults + std::string("startup config fault,");
    error_level = 1;
  }

  //More serious faults 
  if (status & fault_def.FAULT_OVERHEAT) {
    faults = faults + std::string("overheat,");
    error_level = 2;
  }

  if (status & fault_def.FAULT_OVERVOLTAGE) {
    faults = faults + std::string("overvoltage,");
    error_level = 2;
  }

  if (status & fault_def.FAULT_SHORT_CIRCUIT) {
    faults = faults + std::string("short circuit,");
    error_level = 2;
  } 

  if (status & fault_def.FAULT_MOSFET_FAILURE) {
    faults = faults + std::string("mosfet failure,");
    error_level = 2;
  }

}

void MotorsMonitor::motor_feedback(const roboteq_msgs::FeedbackConstPtr& msg, const int motor_num) {
  last_received_feedback_[motor_num] = msg;
}

void MotorsMonitor::motor_status(const roboteq_msgs::StatusConstPtr& msg, const int motor_num) {
  last_received_status_[motor_num] = msg;
}

