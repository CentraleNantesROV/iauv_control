#ifndef IAUV_CONTROL_CONTROLLER_IO_H
#define IAUV_CONTROL_CONTROLLER_IO_H

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
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
using geometry_msgs::msg::Wrench;
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
    Eigen::Quaterniond q;

    Pose() {}
    Pose(const Eigen::Vector3d &t, const Eigen::Quaterniond &q) : t(t), q(q) {}

    inline static void tf2Rotation(const geometry_msgs::msg::Quaternion &q, Eigen::Quaterniond &qe)
    {
      qe.x() = q.x;
      qe.y() = q.y;
      qe.z() = q.z;
      qe.w() = q.w;
      qe.normalize();
    }
    inline Pose operator*(const Pose &other) const
    {
      return {t + q*other.t, q*other.q};
    }
    inline void rotate2(const Vector6d &src, Vector6d &dst) const
    {
      dst.head<3>() = q*src.head<3>();
      dst.tail<3>() = q*src.tail<3>();
    }
    template<class Translation>
    void from(const Translation &t, const geometry_msgs::msg::Quaternion &q)
    {
      this->t.x() = t.x;
      this->t.y() = t.y;
      this->t.z() = t.z;
      tf2Rotation(q, this->q);
    }
    Vector6d toSE3() const;
  };

  template<class Setpoint>
  struct StampedSetpoint : public Setpoint
  {
    std::string frame;
    double time{0};
  };

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  Pose relPose(const std::string &frame);

  // setpoints
  std::string control_frame;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub;
  rclcpp::Subscription<TwistStamped>::SharedPtr vel_sub;
  rclcpp::Subscription<Wrench>::SharedPtr wrench_sub;

  StampedSetpoint<Pose> pose_setpoint;
  StampedSetpoint<Vector6d> vel_setpoint;
  StampedSetpoint<Vector6d> wrench_setpoint;

  // odom estim
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
  bool state_ok{false};
  Vector6d vel;
  Eigen::Quaterniond orientation;

  // command
  std::chrono::milliseconds cmd_period;
  bool use_feedforward = false;
  rclcpp::Service<ControlMode>::SharedPtr control_srv;
  decltype (ControlMode::Request::mode) control_mode;
  rclcpp::TimerBase::SharedPtr cmd_timer;
  void publishThrust();  
  ThrusterAllocator allocator;

  // actual-controller-dependent
  virtual Vector6d computeWrench(const Vector6d &se3_error,
                                 const Vector6d &vel,
                                 const Vector6d &vel_setpoint) = 0;

  // timeouts
  double pose_setpoint_timeout{1};
  double vel_setpoint_timeout{.1};
};
}

#endif // IAUV_CONTROL_CONTROLLER_IO_H
