
#include <ros/ros.h>

namespace grizzly_msgs {
ROS_DECLARE_MESSAGE(Drive);
}

namespace diagnostic_updater {
class DiagnosticStatusWrapper;
}

class EncodersMonitor {
public:
  EncodersMonitor();

  bool ok();
  void diagnostic(diagnostic_updater::DiagnosticStatusWrapper&);

protected:
  ros::NodeHandle nh_;

  // Callbacks receive inbound data
  void encoders_callback(const grizzly_msgs::DriveConstPtr&);
  void drive_callback(const grizzly_msgs::DriveConstPtr&);
  ros::Subscriber sub_encoders_, sub_drive_;

  grizzly_msgs::DriveConstPtr last_received_encoders_;
  grizzly_msgs::DriveConstPtr last_received_drive_;

  ros::Duration encoders_timeout_;
};
