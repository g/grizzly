
#include <ros/ros.h>

namespace grizzly_msgs { ROS_DECLARE_MESSAGE(Drive); }

namespace diagnostic_updater {
class DiagnosticStatusWrapper;
}

class EncodersMonitor {
public:
  EncodersMonitor();
  void initROS(ros::NodeHandle& nh);

  bool ok();
  void diagnostic(diagnostic_updater::DiagnosticStatusWrapper&);

protected:
  bool detectFailedEncoder();
  bool detectFailedEncoderCandidate();

  // Callbacks receive inbound data
  void encodersCallback(const grizzly_msgs::DriveConstPtr&);
  void driveCallback(const grizzly_msgs::DriveConstPtr&);
  ros::Subscriber sub_encoders_, sub_drive_;

  grizzly_msgs::DriveConstPtr last_received_encoders_;
  grizzly_msgs::DriveConstPtr last_received_drive_;

  ros::Duration encoders_timeout_;
};
