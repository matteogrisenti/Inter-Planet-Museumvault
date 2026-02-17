#include <memory>
#include <algorithm>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class GenericAction : public plansys2::ActionExecutorClient
{
public:
  GenericAction(const std::string& action_name, float duration_seconds)
  : plansys2::ActionExecutorClient(action_name, 500ms)
  {
    total_steps_ = static_cast<int>(duration_seconds / 0.5);
    progress_step_ = 1.0f / static_cast<float>(total_steps_);
    progress_ = 0.0f;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0f) {
      progress_ += progress_step_;
      send_feedback(progress_, "Action in progress");
    } else {
      finish(true, 1.0, "Action completed successfully");
      progress_ = 0.0f;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "[" << this->get_name() << "] ... [" 
              << std::min(100.0f, progress_ * 100.0f) << "%]  " 
              << std::flush;
  }

  float progress_;
  float progress_step_;
  int total_steps_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Default values
  std::string action_name = "generic_action";
  float duration = 5.0f; // Default 5 seconds
  
  // Parse command line arguments
  if (argc > 1) {
    action_name = argv[1];
  }
  if (argc > 2) {
    try {
      duration = std::stof(argv[2]);
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("action_node"), 
                  "Invalid duration argument, using default 5.0s");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("action_node"), 
              "Starting action node: %s (duration: %.1fs)", 
              action_name.c_str(), duration);

  auto node = std::make_shared<GenericAction>(action_name, duration);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
