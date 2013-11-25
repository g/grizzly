
#include <ros/ros.h>
#include <string>

namespace roboteq_msgs {
ROS_DECLARE_MESSAGE(Feedback);
ROS_DECLARE_MESSAGE(Status);
}

namespace diagnostic_updater {
class DiagnosticStatusWrapper;
}

class MotorsMonitor {
public:
  MotorsMonitor();

  bool ok();
  void diagnostic(diagnostic_updater::DiagnosticStatusWrapper&);

protected:
  ros::NodeHandle nh_;

  // Callbacks receive inbound data
  void motor_feedback(const roboteq_msgs::FeedbackConstPtr, const int);
  void motor_status(const roboteq_msgs::StatusConstPtr&, const int);
  void msg_watchdog(const ros::TimerEvent&);
  int lookForSeriousFault(uint8_t, diagnostic_updater::DiagnosticStatusWrapper&, const int);

  ros::Subscriber fb_sub_[4], stat_sub_[4];
  ros::Publisher estop_pub_;  
  ros::Duration motors_timeout_;
  roboteq_msgs::StatusConstPtr last_received_status_[4]; 
  roboteq_msgs::FeedbackConstPtr last_received_feedback_[4];
};
