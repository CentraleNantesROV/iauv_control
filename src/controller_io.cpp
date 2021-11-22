#include <iauv_control/controller_io.h>
#include <Eigen/Geometry>

#include <chrono>

using namespace iauv_control;
using namespace std::chrono_literals;
using ControlMode = srv::Control::Request;

void twist2Eigen(const geometry_msgs::msg::Twist &twist, Vector6d &vel)
{
  vel[0] = twist.linear.x;
  vel[1] = twist.linear.y;
  vel[2] = twist.linear.z;
  vel[3] = twist.angular.x;
  vel[4] = twist.angular.y;
  vel[5] = twist.angular.z;
}

void tf2Rotation(const geometry_msgs::msg::Quaternion &q,
                            Eigen::Matrix3d &orientation)
{
  orientation = Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
}

void tf2Eigen(const geometry_msgs::msg::Vector3 &t, const geometry_msgs::msg::Quaternion &q,
              Eigen::Isometry3d &pose)
{
  pose.translation().x() = t.x;
  pose.translation().y() = t.y;
  pose.translation().z() = t.z;
  pose.linear() = Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
}

ControllerIO::ControllerIO(std::string name, rclcpp::NodeOptions options)
  : Node(name,
         options.automatically_declare_parameters_from_overrides(true)
         .allow_undeclared_parameters(true)),
    tf_buffer(this->get_clock()), tf_listener(tf_buffer), allocator(this)
{

  control_frame = get_namespace();
  control_frame = declare_parameter<std::string>("control_frame", control_frame.substr(1) + "/base_link");


  use_feedforward = declare_parameter<bool>("use_feedforward", false);

  pose_sub = create_subscription<PoseStamped>("cmd_pose", 10, [&](PoseStamped::SharedPtr msg)
  {poseSetpointCallback(*msg);});
  vel_sub = create_subscription<TwistStamped>("cmd_vel", 10, [&](TwistStamped::SharedPtr msg)
  {velSetpointCallback(*msg);});
  odom_sub = create_subscription<Odometry>("odom", 10, [&](Odometry::SharedPtr msg)
  {twist2Eigen(msg->twist.twist, vel);
             tf2Rotation(msg->pose.pose.orientation, orientation);
});

  control_srv = create_service<srv::Control>
                ("control_mode",
                 [&](const srv::Control::Request::SharedPtr request,
                 [[maybe_unused]] srv::Control::Response::SharedPtr response)
  {
    control_mode = request->mode;
  });

  cmd_period = std::chrono::milliseconds(declare_parameter<int>("control_period_ms", 50));
  cmd_timer = create_wall_timer(cmd_period, [&](){publishThrust();});

}

void ControllerIO::publishThrust()
{
  // correct setpoints according to timestamps
  const auto now_s{get_clock()->now().seconds()};
  if(pose_setpoint_time - now_s > pose_setpoint_timeout)
    pose_error.setZero();

  if(vel_setpoint_time - now_s > vel_setpoint_timeout)
    vel_setpoint.setZero();

  // check control mode
  if(control_mode == ControlMode::VELOCITY)
  {
    pose_error.setZero();
  }
  else if(control_mode == ControlMode::DEPTH)
  {
    // position mode only for Z, roll, pitch
    pose_error.translation.x() = 0;
    pose_error.translation.y() = 0;
    pose_error.orientation.z() = 0;
  }

  const auto wrench{computeWrench()};

  if(use_feedforward)
    allocator.publish(wrench, orientation, vel);
  else
    allocator.publish(wrench);
}

Eigen::Isometry3d ControllerIO::relPose(const std::string &frame)
{
  static Eigen::Isometry3d rel_pose;

  static geometry_msgs::msg::Transform tf;
  if(tf_buffer.canTransform(control_frame, frame, tf2::TimePointZero, 10ms))
  {
    tf = tf_buffer.lookupTransform(control_frame, frame, tf2::TimePointZero, 10ms).transform;
    tf2Eigen(tf.translation, tf.rotation, rel_pose);
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
    pose_error.translation = rel_pose * pose_error.translation;
    pose_error.orientation = rel_pose.linear() * pose_error.orientation;
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
    const auto R{relPose(twist.header.frame_id).linear()};
    vel_setpoint.head<3>() = R * vel_setpoint.head<3>();
    vel_setpoint.tail<3>() = R * vel_setpoint.tail<3>();
  }
  vel_setpoint_time = get_clock()->now().seconds();
}
