#include <memory>
#include <algorithm>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

// PDDL duration = 4 → period = 4 * 100ms = 400ms

class PutInPodSlot1 : public plansys2::ActionExecutorClient
{
public:
  PutInPodSlot1()
  : plansys2::ActionExecutorClient("put_in_pod_slot_1", 400ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.2;
      send_feedback(progress_, "Put in pod slot 1 running");
    } else {
      finish(true, 1.0, "Put in pod slot 1 completed");
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Putting in pod ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PutInPodSlot1>();

  node->set_parameter(rclcpp::Parameter("action_name", "put_in_pod_slot_1"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
