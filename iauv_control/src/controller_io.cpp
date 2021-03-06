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

void wrench2Eigen(const geometry_msgs::msg::Wrench &wrench, Vector6d &vec)
{
  vec[0] = wrench.force.x;
  vec[1] = wrench.force.y;
  vec[2] = wrench.force.z;
  vec[3] = wrench.torque.x;
  vec[4] = wrench.torque.y;
  vec[5] = wrench.torque.z;
}

Vector6d ControllerIO::Pose::toSE3() const
{
  Vector6d se3_error;
  se3_error.head<3>() = t;

  const auto sin_theta_over_2{q.vec().norm()};
  if(sin_theta_over_2 < 1e-6)
    se3_error.tail<3>().setZero();
  else
  {
    auto angle{2*atan2(sin_theta_over_2, q.w())};
    if(angle > M_PI)  angle -= 2*M_PI;
    else if(angle < -M_PI)  angle += 2*M_PI;
    se3_error.tail<3>() = q.vec() * angle/sin_theta_over_2;
  }
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
  {          pose_setpoint.from(msg->pose.position, msg->pose.orientation);
             pose_setpoint.frame = msg->header.frame_id;
             pose_setpoint.time = get_clock()->now().seconds();});

  vel_sub = create_subscription<TwistStamped>("cmd_vel", 10, [&](TwistStamped::SharedPtr msg)
  {
            twist2Eigen(msg->twist, vel_setpoint);
            vel_setpoint.frame = msg->header.frame_id;
            vel_setpoint.time = get_clock()->now().seconds();});

  wrench_sub = create_subscription<Wrench>("cmd_wrench", 1, [&](Wrench::SharedPtr msg)
  {
               wrench2Eigen(*msg, wrench_setpoint);
               wrench_setpoint.time = get_clock()->now().seconds();});

  odom_sub = create_subscription<Odometry>("pose_gt", 10, [&](Odometry::SharedPtr msg)
  {twist2Eigen(msg->twist.twist, vel);
             Pose::tf2Rotation(msg->pose.pose.orientation, orientation);});

  control_srv = create_service<ControlMode>
                ("control_mode",
                 [&](const ControlMode::Request::SharedPtr request,
                 [[maybe_unused]] ControlMode::Response::SharedPtr response)
  {
    control_mode = request->mode;});

  int cmd_period_ms;
  declare_param_if_needed(this, "control_period_ms", cmd_period_ms, 200);
  cmd_period = std::chrono::milliseconds(cmd_period_ms);
  cmd_timer = create_wall_timer(cmd_period, [&](){publishThrust();});
}

void ControllerIO::publishThrust()
{
  // correct setpoints according to timestamps
  const auto now_s{get_clock()->now().seconds()};
  const auto pose_timeout{now_s - pose_setpoint.time > pose_setpoint_timeout};
  const auto vel_timeout{now_s - vel_setpoint.time  > vel_setpoint_timeout};
  const auto wrench_timeout{now_s - wrench_setpoint.time > pose_setpoint_timeout};

  if(pose_timeout && vel_timeout && wrench_timeout)
  {
    // no more setpoints
    allocator.stop();
    return;
  }

  if(!wrench_timeout)
  {
    // manual wrench control overrides navigation
    if(use_feedforward)
      allocator.publish(wrench_setpoint, orientation, vel);
    else
      allocator.publish(wrench_setpoint);
    return;
  }

  // pose / vel control
  Vector6d se3_error;
  if(pose_timeout || control_mode == ControlMode::Request::VELOCITY)
    se3_error.setZero();
  else if(pose_setpoint.frame == control_frame)
    se3_error = pose_setpoint.toSE3();
  else // to vehicle frame
    se3_error = (relPose(pose_setpoint.frame) * pose_setpoint).toSE3();

  Vector6d vel_setpoint_local;
  if(vel_timeout)
    vel_setpoint_local.setZero();
  else if(vel_setpoint.frame == control_frame || vel_setpoint.isApproxToConstant(0, 1e-3))
    vel_setpoint_local = vel_setpoint;
  else // to vehicle frame
    relPose(vel_setpoint.frame).rotate2(vel_setpoint, vel_setpoint_local);

  // hybrid control mode
  if(control_mode == ControlMode::Request::DEPTH)
  {
    // position mode only for Z, roll, pitch
    se3_error[0] = se3_error[1] = se3_error[5] = 0.;
  }

  const auto wrench{computeWrench(se3_error, vel, vel_setpoint_local)};

  if(use_feedforward)
    allocator.publish(wrench, orientation, vel);
  else
    allocator.publish(wrench);
}

ControllerIO::Pose ControllerIO::relPose(const std::string &frame)
{
  Pose rel_pose;  
  if(tf_buffer.canTransform(control_frame, frame, tf2::TimePointZero, 10ms))
  {
    const auto tf{tf_buffer.lookupTransform(control_frame, frame, tf2::TimePointZero, 10ms).transform};
    rel_pose.from(tf.translation, tf.rotation);
  }
  return rel_pose;
}

