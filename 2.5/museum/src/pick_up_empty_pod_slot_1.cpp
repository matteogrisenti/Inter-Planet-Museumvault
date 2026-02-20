#include <memory>
#include <algorithm>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

// PDDL duration = 2 → period = 2 * 100ms = 200ms

class PickUpEmptyPodSlot1 : public plansys2::ActionExecutorClient
{
public:
  PickUpEmptyPodSlot1()
  : plansys2::ActionExecutorClient("pick_up_empty_pod_slot_1", 200ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.5;
      send_feedback(progress_, "Pick up empty pod slot 1 running");
    } else {
      finish(true, 1.0, "Pick up empty pod slot 1 completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Picking up ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickUpEmptyPodSlot1>();

  node->set_parameter(rclcpp::Parameter("action_name", "pick_up_empty_pod_slot_1"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
