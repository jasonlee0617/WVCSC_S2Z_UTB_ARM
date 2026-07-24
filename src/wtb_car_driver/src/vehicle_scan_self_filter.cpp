#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "wtb_car_driver/vehicle_scan_self_filter.hpp"

namespace
{

class VehicleScanSelfFilter : public rclcpp::Node
{
public:
  VehicleScanSelfFilter()
  : Node("vehicle_scan_self_filter"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    const auto input_topic = this->declare_parameter<std::string>(
      "input_topic", "/scan_unfiltered");
    const auto output_topic = this->declare_parameter<std::string>("output_topic", "/scan");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    transform_timeout_sec_ = this->declare_parameter<double>("transform_timeout_sec", 0.05);
    bounds_.min_x = this->declare_parameter<double>("mask_min_x", -0.825);
    bounds_.max_x = this->declare_parameter<double>("mask_max_x", 0.825);
    bounds_.min_y = this->declare_parameter<double>("mask_min_y", -0.60);
    bounds_.max_y = this->declare_parameter<double>("mask_max_y", 0.60);

    if (base_frame_.empty() || transform_timeout_sec_ < 0.0 ||
      bounds_.min_x >= bounds_.max_x || bounds_.min_y >= bounds_.max_y)
    {
      throw std::invalid_argument("vehicle scan self-filter parameters are invalid");
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&VehicleScanSelfFilter::onScan, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(),
      "Filtering %s -> %s using %s self mask x=[%.3f, %.3f], y=[%.3f, %.3f]",
      input_topic.c_str(), output_topic.c_str(), base_frame_.c_str(),
      bounds_.min_x, bounds_.max_x, bounds_.min_y, bounds_.max_y);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, scan->header.frame_id, scan->header.stamp,
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Not publishing filtered scan: %s -> %s transform unavailable: %s",
        scan->header.frame_id.c_str(), base_frame_.c_str(), error.what());
      return;
    }

    const auto & rotation = transform.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
    auto filtered = *scan;
    wtb_car_driver::maskSelfReturns(
      filtered, transform.transform.translation.x, transform.transform.translation.y,
      yaw, bounds_);
    publisher_->publish(filtered);
  }

  std::string base_frame_;
  double transform_timeout_sec_{0.05};
  wtb_car_driver::SelfMaskBounds bounds_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleScanSelfFilter>());
  rclcpp::shutdown();
  return 0;
}
