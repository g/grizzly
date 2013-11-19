
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "grizzly_msgs/Drive.h"

#include <Eigen/Core>

using Eigen::Vector2f;

class DeadReckoning
{
public:
  DeadReckoning() : nh_(""), max_dt_(0.2)
  {
  }

  /**
   * Separate initialization function for ROS RPC stuff, to make the rest of the class
   * more testable.
   */ 
  void initializeROS()
  {
    ros::param::get("vehicle_width", width_);
    ros::param::get("wheel_radius", radius_);

    pub_ = nh_.advertise<nav_msgs::Odometry>("odom_encoder", 1);
    sub_ = nh_.subscribe("encoders", 1, &DeadReckoning::encodersCallback, this);
  }

protected:
  void encodersCallback(const grizzly_msgs::DriveConstPtr&);
  bool nextEncoders(const grizzly_msgs::DriveConstPtr& encoders, nav_msgs::Odometry* odom);

  // ROS interface
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  // Maintaining state between encoder updates.
  ros::Time last_time_;
  Vector2f last_vels_;

  geometry_msgs::Point position_; 
  geometry_msgs::Twist twist_; 
  double yaw_;

  // Configuration
  double width_, radius_;
  ros::Duration max_dt_;
};
