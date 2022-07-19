#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/algo_list.hpp"
#include "protocol/srv/algo_manager.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client");
  rclcpp::Client<protocol::srv::AlgoManager>::SharedPtr client =
    node->create_client<protocol::srv::AlgoManager>("algo_manager");

  auto request = std::make_shared<protocol::srv::AlgoManager::Request>();
  protocol::msg::AlgoList algo;
  algo.algo_module = protocol::msg::AlgoList::ALGO_FACE;
  request->algo_enable.push_back(algo);
  algo.algo_module = protocol::msg::AlgoList::ALGO_BODY;
  request->algo_enable.push_back(algo);
  algo.algo_module = protocol::msg::AlgoList::ALGO_REID;
  request->algo_enable.push_back(algo);
  request->open_age = true;
  request->open_emotion = true;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "enable result: %d", result.get()->result_enable);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
