#include <iauv_control/controller_io.h>
#include <iauv_control/multi_cascaded_pid.h>

#include <sensor_msgs/msg/joint_state.hpp>

namespace iauv_control
{

using std::string;
using std::vector;

class BodyPID : public ControllerIO, public MultiCascadedPID
{

  sensor_msgs::msg::JointState js;
public:
  BodyPID() : ControllerIO("body_control"), MultiCascadedPID(this)
  {
    const auto ns = string{get_namespace()};

    initControllers(ns, cmd_period);
  }


  Vector6d computeWrench() override
  {
    // call PIDs


    return {};
  }

private:

  void initFromModel([[maybe_unused]] const string &ns) override
  {
    const auto max_wrench{allocator.maxWrench()};

    const std::array<string,6> axes{{"x","y","z","roll","pitch","yaw"}};

    for(size_t i = 0; i < 6; ++i)
    {
      const auto &axis{axes[i]};

      if(max_wrench[i] > 1e-3)
      {
        // the AUV can move in this direction
        pids.emplace_back(axis, 1., 0.4, 0.1, 0, std::numeric_limits<double>::max(), max_wrench[i]);
      }
    }
  }
};
}



// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iauv_control::BodyPID>());
  rclcpp::shutdown();
  return 0;
}
