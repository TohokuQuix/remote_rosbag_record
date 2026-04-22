// Copyright 2025 Quix, Tohoku University
// ROS 2 Jazzy port of remote_rosbag_record/trigger
//
// One-shot node: discovers all Trigger services whose names match ~regex
// and calls each one.  Exits immediately after.
//
// Parameters:
//   regex    string  (required) — regex matched against full service names
//   verbose  bool    true

#include <chrono>
#include <regex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("remote_rosbag_trigger");

  node->declare_parameter<std::string>("regex", "");
  node->declare_parameter<bool>("verbose", true);

  const std::string regex_str = node->get_parameter("regex").as_string();
  if (regex_str.empty()) {
    RCLCPP_ERROR(node->get_logger(), "param 'regex' is required");
    rclcpp::shutdown();
    return 1;
  }

  const bool verbose = node->get_parameter("verbose").as_bool();
  const std::regex expression(regex_str);

  // Brief discovery spin so remote services are visible
  rclcpp::spin_some(node);

  const auto services = node->get_service_names_and_types();

  std::size_t n_match = 0, n_success = 0;
  for (const auto & [name, types] : services) {
    if (!std::regex_match(name, expression)) continue;

    bool is_trigger = false;
    for (const auto & t : types) {
      if (t == "std_srvs/srv/Trigger") { is_trigger = true; break; }
    }
    if (!is_trigger) continue;
    ++n_match;

    auto client = node->create_client<std_srvs::srv::Trigger>(name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(node->get_logger(), "Service '%s' not available", name.c_str());
      continue;
    }

    auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());

    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "Call to '%s' timed out", name.c_str());
      continue;
    }

    ++n_success;
    if (verbose) {
      RCLCPP_INFO(node->get_logger(), "Called '%s': %s",
        name.c_str(), future.get()->message.c_str());
    }
  }

  if (verbose && n_match == 0) {
    RCLCPP_WARN(node->get_logger(), "No Trigger service matched '%s'", regex_str.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
