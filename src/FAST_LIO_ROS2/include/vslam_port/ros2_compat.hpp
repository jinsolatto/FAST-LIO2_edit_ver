#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Small ROS 1-shaped adapter used only while porting this node.  It keeps the
// algorithm independent of rclcpp plumbing and translates ROS 1 parameter
// paths ("Group/key") to ROS 2 parameter names ("Group.key").
namespace ros {
class Time {
public:
  Time() = default;
  explicit Time(const rclcpp::Time &time) : time_(time) {}
  static Time now() { return Time(rclcpp::Clock(RCL_ROS_TIME).now()); }
  double toSec() const { return time_.seconds(); }
  builtin_interfaces::msg::Time to_msg() const {
    builtin_interfaces::msg::Time result;
    const auto nanoseconds = time_.nanoseconds();
    result.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    result.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return result;
  }
private:
  rclcpp::Time time_{0, 0, RCL_ROS_TIME};
};

class Publisher {
public:
  template<typename MessageT> void publish(const MessageT &message) const {
    if (publish_) publish_(std::make_shared<MessageT>(message));
  }
  explicit operator bool() const { return static_cast<bool>(publish_); }
private:
  template<typename MessageT> friend class PublisherFactory;
  friend class NodeHandle;
  std::function<void(std::shared_ptr<void>)> publish_;
};

class Subscriber {};

class NodeHandle {
public:
  explicit NodeHandle(const std::shared_ptr<rclcpp::Node> &node) : node_(node) {}

  template<typename MessageT>
  Publisher advertise(const std::string &topic, size_t depth) {
    auto publisher = node_->create_publisher<MessageT>(topic, rclcpp::QoS(depth));
    Publisher result;
    result.publish_ = [publisher](std::shared_ptr<void> message) {
      publisher->publish(*std::static_pointer_cast<MessageT>(message));
    };
    return result;
  }

  template<typename MessageT, typename CallbackT>
  Subscriber subscribe(const std::string &topic, size_t depth, CallbackT callback) {
    auto subscription = node_->create_subscription<MessageT>(
        topic, rclcpp::QoS(depth), callback);
    subscriptions_.push_back(subscription);
    return Subscriber{};
  }

  template<typename T>
  void param(const std::string &name, T &value, const T &default_value) {
    const auto ros2_name = parameter_name(name);
    if (!node_->has_parameter(ros2_name)) node_->declare_parameter<T>(ros2_name, default_value);
    node_->get_parameter(ros2_name, value);
  }

  bool ok() const { return rclcpp::ok(); }
  std::shared_ptr<rclcpp::Node> node() const { return node_; }

private:
  static std::string parameter_name(std::string name) {
    std::replace(name.begin(), name.end(), '/', '.');
    return name;
  }
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

inline void init(int argc, char **argv, const std::string &) { rclcpp::init(argc, argv); }
inline bool ok() { return rclcpp::ok(); }
inline void spinOnce(NodeHandle &node) { rclcpp::spin_some(node.node()); }
inline void spin(NodeHandle &node) { rclcpp::spin(node.node()); }
inline double time_seconds(const builtin_interfaces::msg::Time &time) {
  return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
}
inline builtin_interfaces::msg::Time time_message(double seconds) {
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(seconds);
  result.nanosec = static_cast<uint32_t>((seconds - result.sec) * 1e9);
  return result;
}
}  // namespace ros
