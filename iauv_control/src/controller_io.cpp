#include <iauv_control/controller_io.h>
#include <iauv_control/param_overrides.h>

#include <Eigen/Geometry>

#include <chrono>

using namespace iauv_control;
using namespace std::chrono_literals;

void twist2Eigen(const geometry_msgs::msg::Twist &twist, Vector6d &vel)
{
  vel[0] = twist.linear.x;
  vel[1] = twist.linear.y;
  vel[2] = twist.linear.z;
  vel[3] = twist.angular.x;
  vel[4] = twist.angular.y;
  vel[5] = twist.angular.z;
}

Vector6d ControllerIO::Pose::toSE3() const
{
  Vector6d se3_error;
  se3_error.head<3>() = t;
  const Eigen::AngleAxisd aa{R};
  se3_error.tail<3>() = aa.axis() * aa.angle();
  return se3_error;
}

ControllerIO::ControllerIO(std::string name, rclcpp::NodeOptions options)
  : Node(name,
         options.automatically_declare_parameters_from_overrides(true)
         .allow_undeclared_parameters(true)),
    tf_buffer(this->get_clock()), tf_listener(tf_buffer), allocator(this)
{

  control_frame = get_namespace();
  declare_param_if_needed(this, "control_frame", control_frame, control_frame.substr(1) + "/base_link");

  declare_param_if_needed(this, "use_feedforward", use_feedforward, false);

  pose_sub = create_subscription<PoseStamped>("cmd_pose", 10, [&](PoseStamped::SharedPtr msg)
  {poseSetpointCallback(*msg);});
  vel_sub = create_subscription<TwistStamped>("cmd_vel", 10, [&](TwistStamped::SharedPtr msg)
  {velSetpointCallback(*msg);});
  odom_sub = create_subscription<Odometry>("pose_gt", 10, [&](Odometry::SharedPtr msg)
  {twist2Eigen(msg->twist.twist, vel);
             Pose::tf2Rotation(msg->pose.pose.orientation, orientation);
});

  control_srv = create_service<ControlMode>
                ("control_mode",
                 [&](const ControlMode::Request::SharedPtr request,
                 [[maybe_unused]] ControlMode::Response::SharedPtr response)
  {
    control_mode = request->mode;
  });

  int cmd_period_ms;
  declare_param_if_needed(this, "control_period_ms", cmd_period_ms, 100);
  cmd_period = std::chrono::milliseconds(cmd_period_ms);
  cmd_timer = create_wall_timer(cmd_period, [&](){publishThrust();});

}

void ControllerIO::publishThrust()
{
  // correct setpoints according to timestamps
  const auto now_s{get_clock()->now().seconds()};
  const auto pose_timeout{now_s - pose_setpoint_time > pose_setpoint_timeout};
  const auto vel_timeout{now_s - vel_setpoint_time  > vel_setpoint_timeout};

  auto se3_error{pose_error.toSE3()};

  if(pose_timeout)
    se3_error.setZero();

  if(vel_timeout)
    vel_setpoint.setZero();

  if(pose_timeout && vel_timeout)
  {
    // no more setpoints: stop thrusters
    allocator.stop();
    return;
  }

  // check control mode
  if(control_mode == ControlMode::Request::VELOCITY)
  {
    se3_error.setZero();
  }
  else if(control_mode == ControlMode::Request::DEPTH)
  {
    // position mode only for Z, roll, pitch
    se3_error[0] = se3_error[1] = se3_error[5] = 0.;
  }

  const auto wrench{computeWrench(se3_error)};

  if(use_feedforward)
    allocator.publish(wrench, orientation, vel);
  else
    allocator.publish(wrench);
}

ControllerIO::Pose ControllerIO::relPose(const std::string &frame)
{
  static Pose rel_pose;

  static geometry_msgs::msg::Transform tf;
  if(tf_buffer.canTransform(control_frame, frame, tf2::TimePointZero, 10ms))
  {
    tf = tf_buffer.lookupTransform(control_frame, frame, tf2::TimePointZero, 10ms).transform;
    rel_pose.from(tf.translation, tf.rotation);
  }
  return rel_pose;
}

void ControllerIO::poseSetpointCallback(const PoseStamped &pose)
{
  // to Eigen
  pose_error.from(pose.pose.position, pose.pose.orientation);
  // to vehicle frame if needed
  if(pose.header.frame_id != control_frame)
  {
    const auto rel_pose{relPose(pose.header.frame_id)};
    pose_error.changeFrame(rel_pose);
  }
  pose_setpoint_time = get_clock()->now().seconds();
}

void ControllerIO::velSetpointCallback(const TwistStamped &twist)
{
  // to Eigen
  twist2Eigen(twist.twist, vel_setpoint);

  // to vehicle frame if needed
  if(twist.header.frame_id != control_frame)
  {
    const auto R{relPose(twist.header.frame_id).R};
    vel_setpoint.head<3>() = R * vel_setpoint.head<3>();
    vel_setpoint.tail<3>() = R * vel_setpoint.tail<3>();
  }
  vel_setpoint_time = get_clock()->now().seconds();
}
