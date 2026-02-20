#include <memory>
#include <algorithm>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

// PDDL duration = 14 → period = 14 * 100ms = 1400ms

class CoolArtifactWhileCarryingInPodSlot1 : public plansys2::ActionExecutorClient
{
public:
  CoolArtifactWhileCarryingInPodSlot1()
  : plansys2::ActionExecutorClient("cool_artifact_while_carrying_in_pod_slot_1", 1400ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.05;
      send_feedback(progress_, "Cooling artifact while carrying in pod slot 1 running");
    } else {
      finish(true, 1.0, "Cooling artifact while carrying in pod slot 1 completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Cooling ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoolArtifactWhileCarryingInPodSlot1>();

  node->set_parameter(rclcpp::Parameter("action_name", "cool_artifact_while_carrying_in_pod_slot_1"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
