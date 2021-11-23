#include <iauv_control/thruster_allocator.h>
#include <iauv_control/model_parser.h>
#include <rclcpp/rclcpp.hpp>

using namespace iauv_control;

ThrusterAllocator::ThrusterAllocator(rclcpp::Node *node) : node(node)
{
  // get thrusters TM and max thrusts
  thrusts.name = robot.parseModel(node->get_namespace());

  thrusts.effort.resize(robot.n_thr);

  // init publishers to robot
  cmd_pub = node->create_publisher<sensor_msgs::msg::JointState>("thruster_command",1);

#ifdef WITH_IAUV_GAZEBO_PLUGINS
  for(size_t i = 0; i < robot.n_thr; ++i)
  {
    const auto topic{"thrusters/id_" + std::to_string(i) + "/input"};
    cmd_gz_pub.push_back(node->create_publisher<FloatStamped>(topic,1));
  }
#endif
}


void ThrusterAllocator::publish(const Vector6d &wrench)
{
  const auto now = node->get_clock()->now();

  // embedded part
  robot.solveWrench(wrench, thrusts.effort);

  std::cout << "thrusts: ";
  for(const auto &v: thrusts.effort)
    std::cout << v << " ";
  std::cout << std::endl;

  thrusts.header.set__stamp(now);
  cmd_pub->publish(thrusts);

  // Gz part
#ifdef WITH_IAUV_GAZEBO_PLUGINS
  cmd_gz.header.set__stamp(now);
  for(size_t i = 0; i < robot.n_thr; ++i)
  {
    cmd_gz.set__data(thrusts.effort[i]);
    cmd_gz_pub[i]->publish(cmd_gz);
  }
#endif
}
