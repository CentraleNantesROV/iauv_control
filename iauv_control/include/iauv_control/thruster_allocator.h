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

  inline void stop()
  {
    for(auto &v: thruster_cmd.velocity)
      v = 0;
    publish();
  }
  inline void publish(const Vector6d & wrench)
  {
     robot.solveWrench(wrench, thruster_cmd.velocity);
     publish();
  }
  inline void publish(Vector6d wrench, const Eigen::Quaterniond &q, const Vector6d &vel)
  {
    robot.compensate(wrench, q, vel);
    publish(wrench);
  }

  inline Vector6d maxWrench() const  {return robot.max_wrench;}

private:
  rclcpp::Node *node;

  // model interface
  IAUV robot;

  sensor_msgs::msg::JointState thruster_cmd;
  rclcpp::Publisher<decltype (thruster_cmd)>::SharedPtr cmd_pub;

  void publish();

#ifdef WITH_IAUV_GAZEBO_PLUGINS
  // will forward thrust command to each UUV Gazebo topics
  std::vector<rclcpp::Publisher<FloatStamped>::SharedPtr> cmd_gz_pub;
  FloatStamped cmd_gz;
#endif

};


}


#endif // IAUV_CONTROL_THRUSTER_ALLOCATOR_H
