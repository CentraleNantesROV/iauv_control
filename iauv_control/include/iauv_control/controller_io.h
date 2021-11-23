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
#include <iauv_control_msgs/srv/control.hpp>

namespace iauv_control
{

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using ControlMode = iauv_control_msgs::srv::Control;

class ControllerIO : public rclcpp::Node
{  
public:
  ControllerIO(std::string name, rclcpp::NodeOptions options = rclcpp::NodeOptions{});

protected:

  struct Pose
  {
    Eigen::Vector3d t;
    Eigen::Matrix3d R;

    inline static void tf2Rotation(const geometry_msgs::msg::Quaternion &q, Eigen::Matrix3d &orientation)
    {
      orientation = Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
    }
    inline void changeFrame(const Pose &rel)
    {
      t = rel.t  + rel.R*t;
      R = rel.R*R;
    }
    template<class Translation>
    void from(const Translation &t, const geometry_msgs::msg::Quaternion &q)
    {
      this->t.x() = t.x;
      this->t.y() = t.y;
      this->t.z() = t.z;
      tf2Rotation(q, R);
    }
    Vector6d toSE3() const;
  };

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  Pose relPose(const std::string &frame);

  // setpoints
  std::string control_frame;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub;
  void poseSetpointCallback(const PoseStamped &pose);
  rclcpp::Subscription<TwistStamped>::SharedPtr vel_sub;
  void velSetpointCallback(const TwistStamped &twist);
  Vector6d vel_setpoint;
  Pose pose_error;

  // odom estim
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
  bool state_ok{false};
  Vector6d vel;
  Eigen::Matrix3d orientation;

  // command
  std::chrono::milliseconds cmd_period;
  bool use_feedforward = false;
  rclcpp::Service<ControlMode>::SharedPtr control_srv;
  decltype (ControlMode::Request::mode) control_mode;
  rclcpp::TimerBase::SharedPtr cmd_timer;
  void publishThrust();  
  ThrusterAllocator allocator;

  // actual-controller-dependent
  virtual Vector6d computeWrench(const Vector6d &se3_error) = 0;

  // timeouts
  double pose_setpoint_time{0};
  double vel_setpoint_time{0};
  double pose_setpoint_timeout{1};
  double vel_setpoint_timeout{.1};
};
}

#endif // IAUV_CONTROL_CONTROLLER_IO_H
