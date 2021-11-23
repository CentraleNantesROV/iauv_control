#ifndef IAUV_CONTROL_PARAM_OVERRIDES_H
#define IAUV_CONTROL_PARAM_OVERRIDES_H

#include <rclcpp/node.hpp>

namespace iauv_control
{

template <class Param>
void declare_param_if_needed(rclcpp::Node *node,
                              const std::string &name,
                              Param &param,
                              const Param& default_value)
{
  if(node->has_parameter(name))
    node->get_parameter(name, param);
  else
    param = node->declare_parameter(name, default_value);
}

}

#endif // PARAM_OVERRIDES_H
