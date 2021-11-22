#ifndef IAUV_CONTROL_THRUSTER_ALLOCATOR_H
#define IAUV_CONTROL_THRUSTER_ALLOCATOR_H

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#ifdef WITH_IAUV_GAZEBO_PLUGINS
#include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>
#endif

#include <iauv_control/eigen_typedefs.h>
#include <iauv_control/model_parser.h>

namespace iauv_control
{

#ifdef WITH_IAUV_GAZEBO_PLUGINS
using uuv_gazebo_ros_plugins_msgs::msg::FloatStamped;
#endif

class ThrusterAllocator
{

public:
  ThrusterAllocator(rclcpp::Node *node);

  void publish(const Vector6d & wrench);
  inline void publish(Vector6d wrench, const Eigen::Matrix3d &R, const Vector6d &vel)
  {
    robot.compensate(wrench, R, vel);
    publish(wrench);
  }

  inline Vector6d maxWrench() const  {return robot.max_wrench;}

private:
  rclcpp::Node *node;

  // model interface
  IAUV robot;

  sensor_msgs::msg::JointState thrusts;
  rclcpp::Publisher<decltype (thrusts)>::SharedPtr cmd_pub;

#ifdef WITH_IAUV_GAZEBO_PLUGINS
  // will forward thrust command to each UUV Gazebo topics
  std::vector<rclcpp::Publisher<FloatStamped>::SharedPtr> cmd_gz_pub;
  FloatStamped cmd_gz;
#endif

};


}


#endif // IAUV_CONTROL_THRUSTER_ALLOCATOR_H
