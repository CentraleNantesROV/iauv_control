#include <iauv_control/multi_cascaded_pid.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo_msgs/srv/apply_joint_effort.hpp>
#include <iauv_control/service_sync.h>

#include <iauv_control/model_parser.h>

using namespace std;
using sensor_msgs::msg::JointState;
using gazebo_msgs::srv::ApplyJointEffort;

namespace iauv_control
{

enum class Update{STATE, SETPOINT};

class JointPID : public rclcpp::Node, public MultiCascadedPID
{
public:
  JointPID() : Node("joint_pid",rclcpp::NodeOptions()
                    .automatically_declare_parameters_from_overrides(true)
                    .allow_undeclared_parameters(true)),
    MultiCascadedPID(this),
    gz_client("gz_client")
  {       
    // used for Gazebo joints
    const auto ns = string{get_namespace()};
    if(has_parameter("model_name"))
      get_parameter("model_name", gz_prefix);
    else
      gz_prefix = declare_parameter("model_name", ns.substr(1)) + "::";

    setpoint_sub = create_subscription<JointState>("joint_setpoint", 2, [&](JointState::UniquePtr msg)
    {updateJS(*msg, Update::SETPOINT);});

    state_sub = create_subscription<JointState>("joint_states", 2, [&](JointState::UniquePtr msg)
    {updateJS(*msg, Update::STATE);});

    initControllers(ns, 50ms);

    effort = std::make_shared<ApplyJointEffort::Request>();
    effort->duration.set__nanosec(samplingTime<std::chrono::nanoseconds>().count());
    //gz_client.init("/apply_joint_effort"); // will also wait

    cmd_timer = create_wall_timer(samplingTime(), [&](){updatePIDs();});
  }

private:
  string gz_prefix;

  rclcpp::TimerBase::SharedPtr cmd_timer;

  void updatePIDs()
  {
    for(auto &pid: pids)
    {
      effort->joint_name = gz_prefix + pid.axis;
      effort->effort = pid.update();
      gz_client.sendRequest(effort);
    }
  }

  // in
  rclcpp::Subscription<JointState>::SharedPtr setpoint_sub, state_sub;

  void updateJS(const JointState &js, Update type)
  {
    const auto use_position{js.position.size()};
    const auto use_vel{js.velocity.size()};

    for(size_t i = 0; i < js.name.size(); ++i)
    {
      if(auto pid_opt = whoControls(js.name[i]);pid_opt.has_value())
      {
        auto & pid{*(pid_opt.value())};
        pid.use_position = use_position;
        if(type == Update::STATE)
        {
          if(use_position) pid.position = js.position[i];
          if(use_vel) pid.vel = js.velocity[i];
        }
        else
        {
          if(use_position) pid.position_sp = js.position[i];

          if(use_vel)
            pid.vel_sp = js.velocity[i];
          else
            pid.vel_sp = 0;
        }
      }
    }
  }

  // out
  ApplyJointEffort::Request::SharedPtr effort;
  ServiceNodeSync<ApplyJointEffort> gz_client;

  void initFromModel(const std::string &ns) override
  {
    const auto urdf{IAUV::getModel(ns, "joint_pid")};

    vector<string> axes;

    for(const auto &[name, joint]: urdf->joints_)
    {
      if(joint->type == joint->FIXED)
        continue;

      if(joint->limits && joint->limits->upper == joint->limits->lower)
        continue;

      if(name.find("thruster") != name.npos || name.find("fin") != name.npos)
        continue;

      axes.push_back(name);

      pids.emplace_back(name, 1., 0.4, 0.1, 0, joint->limits->velocity, joint->limits->effort);
    }
  }
};
}

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iauv_control::JointPID>());
  rclcpp::shutdown();
  return 0;
}
