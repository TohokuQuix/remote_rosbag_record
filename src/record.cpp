// Copyright 2025 Quix, Tohoku University
// ROS 2 Jazzy port of remote_rosbag_record/record
//
// Services:
//   ~/start  (std_srvs/srv/Trigger) — begin recording
//   ~/stop   (std_srvs/srv/Trigger) — finalize and close bag
//
// Topics published:
//   ~/is_recording  (std_msgs/msg/Bool, transient_local) — current state
//
// Parameters (all settable before calling start):
//   record_all              bool              false
//   topics                  string[]          []
//   regex                   string            ""
//   exclude_regex           string            ""
//   include_unpublished     bool              true
//   include_hidden          bool              false
//   storage_id              string            "mcap"
//   output_directory        string            ""   (current dir if empty)
//   prefix                  string            ""
//   name                    string            ""   (overrides prefix+date)
//   append_date             bool              true

#include <chrono>
#include <ctime>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rmw/rmw.h>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class RemoteRosbagRecord : public rclcpp::Node
{
public:
  RemoteRosbagRecord()
  : Node("remote_rosbag_record")
  {
    declare_parameter<bool>("record_all", false);
    declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
    declare_parameter<std::string>("regex", "");
    declare_parameter<std::string>("exclude_regex", "");
    declare_parameter<bool>("include_unpublished", true);
    declare_parameter<bool>("include_hidden", false);
    declare_parameter<std::string>("storage_id", "mcap");
    declare_parameter<std::string>("output_directory", "");
    declare_parameter<std::string>("prefix", "");
    declare_parameter<std::string>("name", "");
    declare_parameter<bool>("append_date", true);

    is_recording_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/remote_rosbag_record/is_recording", rclcpp::QoS(1).transient_local());
    publish_state(false);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/remote_rosbag_record/start",
      std::bind(&RemoteRosbagRecord::handle_start, this, _1, _2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/remote_rosbag_record/stop",
      std::bind(&RemoteRosbagRecord::handle_stop,  this, _1, _2));

    RCLCPP_INFO(get_logger(), "Ready — call ~/start to begin recording");
  }

  ~RemoteRosbagRecord()
  {
    stop_recording();
  }

private:
  // ── helpers ────────────────────────────────────────────────────────────────

  void publish_state(bool recording)
  {
    std_msgs::msg::Bool msg;
    msg.data = recording;
    is_recording_pub_->publish(msg);
  }

  std::string build_bag_uri() const
  {
    const std::string name   = get_parameter("name").as_string();
    const std::string prefix = get_parameter("prefix").as_string();
    const std::string outdir = get_parameter("output_directory").as_string();
    const bool append_date   = get_parameter("append_date").as_bool();

    std::string bag_name;
    if (!name.empty()) {
      bag_name = name;
    } else {
      bag_name = prefix;
      if (append_date) {
        const auto now = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M_%S", std::localtime(&t));
        bag_name += buf;
      }
    }

    if (!outdir.empty()) {
      bag_name = outdir + "/" + bag_name;
    }
    return bag_name;
  }

  // ── service handlers ───────────────────────────────────────────────────────

  void handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Received start request");

    if (recorder_) {
      RCLCPP_ERROR(get_logger(), "Already recording");
      res->success = false;
      res->message = "Already recording";
      return;
    }

    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.storage_id = get_parameter("storage_id").as_string();
    storage_opts.uri        = build_bag_uri();

    rosbag2_transport::RecordOptions record_opts;
    record_opts.all_topics              = get_parameter("record_all").as_bool();
    record_opts.topics                  = get_parameter("topics").as_string_array();
    record_opts.regex                   = get_parameter("regex").as_string();
    record_opts.exclude_regex           = get_parameter("exclude_regex").as_string();
    record_opts.include_unpublished_topics = get_parameter("include_unpublished").as_bool();
    record_opts.include_hidden_topics   = get_parameter("include_hidden").as_bool();
    record_opts.rmw_serialization_format = rmw_get_serialization_format();

    RCLCPP_INFO(
      get_logger(),
      "Preparing recorder: storage_id=%s serialization_format=%s uri=%s",
      storage_opts.storage_id.c_str(),
      record_opts.rmw_serialization_format.c_str(),
      storage_opts.uri.c_str());

    try {
      auto writer = std::make_shared<rosbag2_cpp::Writer>();
      recorder_ = std::make_shared<rosbag2_transport::Recorder>(
        writer, storage_opts, record_opts, "rosbag2_recorder");

      record_exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      record_exec_->add_node(recorder_);
      recorder_->record();
      record_thread_ = std::thread([this]() { record_exec_->spin(); });

      publish_state(true);
      RCLCPP_INFO(get_logger(), "Started recording -> %s", storage_opts.uri.c_str());
      res->success = true;
      res->message = "Started -> " + storage_opts.uri;
      RCLCPP_INFO(get_logger(), "Start request accepted -> %s", storage_opts.uri.c_str());
    } catch (const std::exception & e) {
      recorder_.reset();
      RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", e.what());
      res->success = false;
      res->message = std::string("Failed to start recording: ") + e.what();
    } catch (...) {
      recorder_.reset();
      RCLCPP_ERROR(get_logger(), "Failed to start recording with unknown exception");
      res->success = false;
      res->message = "Failed to start recording with unknown exception";
    }
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Received stop request");
    if (!recorder_) {
      RCLCPP_ERROR(get_logger(), "Not recording");
      res->success = false;
      res->message = "Not recording";
      return;
    }

    stop_recording();
    publish_state(false);
    RCLCPP_INFO(get_logger(), "Stopped recording");

    res->success = true;
    res->message = "Stopped recording";
  }

  void stop_recording()
  {
    if (!recorder_) return;
    recorder_->stop();
    if (record_exec_) {
      record_exec_->cancel();
    }
    if (record_thread_.joinable()) {
      record_thread_.join();
    }
    record_exec_.reset();
    recorder_.reset();
  }

  // ── members ────────────────────────────────────────────────────────────────

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    is_recording_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   stop_srv_;

  std::shared_ptr<rosbag2_transport::Recorder>                  recorder_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor>    record_exec_;
  std::thread                                                   record_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemoteRosbagRecord>());
  rclcpp::shutdown();
  return 0;
}
