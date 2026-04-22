// Copyright 2025 Quix, Tohoku University
// ROS 2 Jazzy port of remote_rosbag_record/joy_listener
//
// Subscribes to sensor_msgs/msg/Joy and calls start/stop Trigger services
// on button press (rising-edge only).
//
// Parameters:
//   start_button   int     12  (PS4 R3)
//   stop_button    int     11  (PS4 L3)
//   start_regex    string  ".*/start"
//   stop_regex     string  ".*/stop"
//   verbose        bool    true

#include <chrono>
#include <regex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

using std::placeholders::_1;

class JoyListener : public rclcpp::Node
{
public:
  JoyListener()
  : Node("joy_listener")
  {
    declare_parameter<int>("start_button", 12);
    declare_parameter<int>("stop_button",  11);
    declare_parameter<std::string>("start_regex", ".*/start");
    declare_parameter<std::string>("stop_regex",  ".*/stop");
    declare_parameter<bool>("verbose", true);

    start_button_ = get_parameter("start_button").as_int();
    stop_button_  = get_parameter("stop_button").as_int();
    start_regex_  = std::regex(get_parameter("start_regex").as_string());
    stop_regex_   = std::regex(get_parameter("stop_regex").as_string());
    verbose_      = get_parameter("verbose").as_bool();

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, std::bind(&JoyListener::on_joy, this, _1));

    RCLCPP_INFO(get_logger(), "Listening — start_button=%d stop_button=%d",
      start_button_, stop_button_);
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool start_pressed = button_down(msg->buttons, start_button_);
    if (!start_was_pressed_ && start_pressed) {
      call_matching_services(start_regex_, "start");
    }
    start_was_pressed_ = start_pressed;

    const bool stop_pressed = button_down(msg->buttons, stop_button_);
    if (!stop_was_pressed_ && stop_pressed) {
      call_matching_services(stop_regex_, "stop");
    }
    stop_was_pressed_ = stop_pressed;
  }

  static bool button_down(const std::vector<int32_t> & buttons, int idx)
  {
    return idx >= 0 &&
           static_cast<std::size_t>(idx) < buttons.size() &&
           buttons[idx] > 0;
  }

  void call_matching_services(const std::regex & expr, const char * action_name)
  {
    const auto services = get_service_names_and_types();
    bool any = false;

    for (const auto & [name, types] : services) {
      if (!std::regex_match(name, expr)) continue;

      bool is_trigger = false;
      for (const auto & t : types) {
        if (t == "std_srvs/srv/Trigger") { is_trigger = true; break; }
      }
      if (!is_trigger) continue;
      any = true;

      auto client = create_client<std_srvs::srv::Trigger>(name);
      if (!client->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_ERROR(get_logger(), "Service '%s' not available", name.c_str());
        continue;
      }

      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());

      if (rclcpp::spin_until_future_complete(
            shared_from_this(), future, std::chrono::seconds(3))
          != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Call to '%s' timed out", name.c_str());
        continue;
      }

      if (verbose_) {
        RCLCPP_INFO(get_logger(), "[%s] Called '%s': %s",
          action_name, name.c_str(), future.get()->message.c_str());
      }
    }

    if (verbose_ && !any) {
      RCLCPP_WARN(get_logger(), "No Trigger service matched for '%s'", action_name);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

  int         start_button_;
  int         stop_button_;
  std::regex  start_regex_;
  std::regex  stop_regex_;
  bool        verbose_;

  bool start_was_pressed_{false};
  bool stop_was_pressed_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyListener>());
  rclcpp::shutdown();
  return 0;
}
