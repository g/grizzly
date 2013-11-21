

#include <ros/ros.h>
#include "grizzly_motion/change_limiter.h"

using boost::shared_ptr;

namespace grizzly_msgs {
ROS_DECLARE_MESSAGE(Drive);
ROS_DECLARE_MESSAGE(RawStatus);
}

namespace diagnostic_updater {
class Updater;
class HeaderlessTopicDiagnostic;
}

class EncodersMonitor;

typedef ChangeLimiter<grizzly_msgs::Drive> DriveChangeLimiter;

class MotionSafety
{
public:
  MotionSafety();

protected:
  ros::NodeHandle nh_;

  // Topics directly monitored in this node.
  void driveCallback(const grizzly_msgs::DriveConstPtr&);
  void mcuStatusCallback(const grizzly_msgs::RawStatus&);
  ros::Subscriber sub_drive_, sub_mcu_status_; 

  // Publish cmd_drive through to safe_cmd_drive.
  ros::Publisher pub_safe_drive_;
    
  // Publish light/sound warnings before allowing movement.
  ros::Publisher pub_ambience_;

  // Publish diagnostics for the whole node from here. The update timer takes care of
  // calling the Updater's update method.
  shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  ros::Timer diagnostic_update_timer_;

  // Monitor the frequency of the MCU status and incoming cmd_drive messages for
  // acceptable range.
  double expected_mcu_status_frequency_;
  double min_cmd_drive_freq_, max_cmd_drive_freq_;
  shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> diag_mcu_status_freq_, diag_cmd_drive_freq_;

  // Separate class for monitoring encoders for sanity.
  shared_ptr<EncodersMonitor> encoders_monitor_; 
  shared_ptr<DriveChangeLimiter> accel_limiters_[4];

  double width_;
  double radius_;
  double max_accel_; 
};


