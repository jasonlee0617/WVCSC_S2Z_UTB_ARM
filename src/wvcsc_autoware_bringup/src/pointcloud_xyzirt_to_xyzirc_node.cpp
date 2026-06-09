#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace
{
int field_offset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

float read_float32(const sensor_msgs::msg::PointCloud2 & cloud, const uint8_t * point, int offset)
{
  if (offset < 0 || static_cast<size_t>(offset + sizeof(float)) > cloud.point_step) {
    return 0.0F;
  }

  float value = 0.0F;
  std::memcpy(&value, point + offset, sizeof(float));
  return value;
}

uint16_t read_uint16(const sensor_msgs::msg::PointCloud2 & cloud, const uint8_t * point, int offset)
{
  if (offset < 0 || static_cast<size_t>(offset + sizeof(uint16_t)) > cloud.point_step) {
    return 0U;
  }

  uint16_t value = 0U;
  std::memcpy(&value, point + offset, sizeof(uint16_t));
  return value;
}

uint8_t intensity_to_uint8(float intensity)
{
  if (!std::isfinite(intensity)) {
    return 0U;
  }

  const auto clamped = std::clamp(intensity, 0.0F, 255.0F);
  return static_cast<uint8_t>(std::lround(clamped));
}

sensor_msgs::msg::PointField make_field(
  const std::string & name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}
}  // namespace

class PointCloudXyzirtToXyzircNode : public rclcpp::Node
{
public:
  PointCloudXyzirtToXyzircNode() : Node("pointcloud_xyzirt_to_xyzirc_node")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/sensing/lidar/pointcloud_raw");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/sensing/lidar/pointcloud_autoware");

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, qos,
      std::bind(&PointCloudXyzirtToXyzircNode::on_pointcloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "PointCloud XYZIRT->XYZIRC adapter ready: input=%s output=%s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void on_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input)
  {
    const int x_offset = field_offset(*input, "x");
    const int y_offset = field_offset(*input, "y");
    const int z_offset = field_offset(*input, "z");
    const int intensity_offset = field_offset(*input, "intensity");
    const int ring_offset = field_offset(*input, "ring");
    const int channel_offset = field_offset(*input, "channel");

    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || intensity_offset < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input pointcloud is missing x/y/z/intensity fields; skipping conversion");
      return;
    }

    if (ring_offset < 0 && channel_offset < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input pointcloud is missing ring/channel field; channel will be published as 0");
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = input->header;
    output.height = input->height;
    output.width = input->width;
    output.is_bigendian = input->is_bigendian;
    output.is_dense = input->is_dense;
    output.point_step = 16;
    output.row_step = output.point_step * output.width;
    output.fields = {
      make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
      make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
      make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
      make_field("intensity", 12, sensor_msgs::msg::PointField::UINT8),
      make_field("return_type", 13, sensor_msgs::msg::PointField::UINT8),
      make_field("channel", 14, sensor_msgs::msg::PointField::UINT16),
    };
    output.data.resize(static_cast<size_t>(output.height) * output.row_step);

    const size_t point_count = static_cast<size_t>(input->width) * input->height;
    for (size_t i = 0; i < point_count; ++i) {
      const auto * in_point = input->data.data() + i * input->point_step;
      auto * out_point = output.data.data() + i * output.point_step;

      const float x = read_float32(*input, in_point, x_offset);
      const float y = read_float32(*input, in_point, y_offset);
      const float z = read_float32(*input, in_point, z_offset);
      const uint8_t intensity = intensity_to_uint8(read_float32(*input, in_point, intensity_offset));
      const uint8_t return_type = 1U;
      const uint16_t channel = read_uint16(
        *input, in_point, ring_offset >= 0 ? ring_offset : channel_offset);

      std::memcpy(out_point + 0, &x, sizeof(float));
      std::memcpy(out_point + 4, &y, sizeof(float));
      std::memcpy(out_point + 8, &z, sizeof(float));
      std::memcpy(out_point + 12, &intensity, sizeof(uint8_t));
      std::memcpy(out_point + 13, &return_type, sizeof(uint8_t));
      std::memcpy(out_point + 14, &channel, sizeof(uint16_t));
    }

    publisher_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudXyzirtToXyzircNode>());
  rclcpp::shutdown();
  return 0;
}
