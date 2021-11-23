#ifndef IAUV_CONTROL_CONTROLLER_IO_H
#define IAUV_CONTROL_CONTROLLER_IO_H

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Geometry>
#include <iauv_control/thruster_allocator.h>
#include <iauv_control/srv/control.hpp>

namespace iauv_control
{

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

class ControllerIO : public rclcpp::Node
{  
public:
  ControllerIO(std::string name, rclcpp::NodeOptions options = rclcpp::NodeOptions{});

protected:

  struct PoseError : public Vector6d
  {
    inline void from(const geometry_msgs::msg::Point &t, const geometry_msgs::msg::Quaternion &q)
    {     
      data()[0] = t.x;
      data()[1] = t.y;
      data()[2] = t.z;
      Eigen::AngleAxisd aa{Eigen::Quaterniond(q.w, q.x, q.y, q.z)};
      tail<3>() = aa.axis() * aa.angle();
    }
    inline auto& translation()
    {
      return head<3>();
    }
  };

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  Eigen::Isometry3d relPose(const std::string &frame);

  // setpoints
  std::string control_frame;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub;
  void poseSetpointCallback(const PoseStamped &pose);
  rclcpp::Subscription<TwistStamped>::SharedPtr vel_sub;
  void velSetpointCallback(const TwistStamped &twist);
  Vector6d vel_setpoint;
  PoseError pose_error;

  // odom estim
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
  bool state_ok{false};
  Vector6d vel;
  Eigen::Matrix3d orientation;

  // command
  std::chrono::milliseconds cmd_period;
  bool use_feedforward = false;
  rclcpp::Service<srv::Control>::SharedPtr control_srv;
  decltype (srv::Control_Request::mode) control_mode;
  rclcpp::TimerBase::SharedPtr cmd_timer;
  void publishThrust();  
  ThrusterAllocator allocator;

  // actual-controller-dependent
  virtual Vector6d computeWrench() = 0;

  // timeouts
  double pose_setpoint_time{0};
  double vel_setpoint_time{0};
  double pose_setpoint_timeout{1};
  double vel_setpoint_timeout{.1};
};
}

#endif // IAUV_CONTROL_CONTROLLER_IO_H
