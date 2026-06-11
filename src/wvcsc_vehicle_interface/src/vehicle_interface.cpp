#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <wtb_car_driver/msg/car_msg.hpp>

using namespace std::chrono_literals;

class VehicleInterface : public rclcpp::Node
{
public:
  VehicleInterface() : Node("wvcsc_vehicle_interface")
  {
    max_speed_ = declare_parameter("max_speed", 1.0);
    max_steering_angle_ = declare_parameter("max_steering_angle", 0.6109);
    command_timeout_sec_ = declare_parameter("command_timeout_sec", 0.5);
    publish_twist_cmd_ = declare_parameter("publish_twist_cmd", true);
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    twist_cmd_topic_ = declare_parameter("twist_cmd_topic", std::string("/twist_cmd"));

    control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
      "/control/command/control_cmd", 10,
      std::bind(&VehicleInterface::onControlCmd, this, std::placeholders::_1));
    gear_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
      "/control/command/gear_cmd", 10,
      std::bind(&VehicleInterface::onGearCmd, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/car_odom", 10, std::bind(&VehicleInterface::onOdom, this, std::placeholders::_1));
    car_status_sub_ = create_subscription<wtb_car_driver::msg::CarMsg>(
      "/wtb_car_message", 10,
      std::bind(&VehicleInterface::onCarStatus, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    twist_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_cmd_topic_, 10);
    run_static_pub_ = create_publisher<std_msgs::msg::String>("/run_static", 10);

    velocity_report_pub_ =
      create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
    steering_report_pub_ =
      create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
    control_mode_pub_ =
      create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
    gear_report_pub_ =
      create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10);
    control_mode_srv_ =
      create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
      "/control/control_mode_request",
      std::bind(
        &VehicleInterface::onControlModeRequest, this, std::placeholders::_1,
        std::placeholders::_2));

    current_gear_ = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    current_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    last_control_time_ = now();
    status_timer_ = create_wall_timer(100ms, std::bind(&VehicleInterface::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "WVCSC vehicle interface ready: max_speed=%.2f m/s, max_steer=%.3f rad, output=%s",
      max_speed_, max_steering_angle_, publish_twist_cmd_ ? twist_cmd_topic_.c_str() : cmd_vel_topic_.c_str());
  }

private:
  void onControlCmd(const autoware_control_msgs::msg::Control::SharedPtr msg)
  {
    if (!isAutonomousControlMode()) {
      return;
    }

    last_control_time_ = now();

    const double speed = std::clamp(
      static_cast<double>(msg->longitudinal.velocity), -max_speed_, max_speed_);
    const double steer = std::clamp(
      static_cast<double>(msg->lateral.steering_tire_angle),
      -max_steering_angle_, max_steering_angle_);
    publishChassisCommand(speed, steer);
  }

  void onGearCmd(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg)
  {
    current_gear_ = msg->command;

    if (
      msg->command == autoware_vehicle_msgs::msg::GearCommand::PARK ||
      msg->command == autoware_vehicle_msgs::msg::GearCommand::NEUTRAL) {
      publishChassisCommand(0.0, 0.0);
    }
    publishGearReport();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_velocity_ = msg->twist.twist.linear.x;
    latest_lateral_velocity_ = msg->twist.twist.linear.y;
    latest_heading_rate_ = msg->twist.twist.angular.z;

    autoware_vehicle_msgs::msg::VelocityReport report;
    report.header.stamp = msg->header.stamp;
    report.header.frame_id = "base_link";
    report.longitudinal_velocity = latest_velocity_;
    report.lateral_velocity = latest_lateral_velocity_;
    report.heading_rate = latest_heading_rate_;
    velocity_report_pub_->publish(report);
  }

  void onCarStatus(const wtb_car_driver::msg::CarMsg::SharedPtr msg)
  {
    latest_steering_angle_ = std::clamp(
      static_cast<double>(msg->angle), -max_steering_angle_, max_steering_angle_);
  }

  void onTimer()
  {
    const auto current_time = now();
    const double elapsed = (current_time - last_control_time_).seconds();

    if (isAutonomousControlMode() && elapsed > command_timeout_sec_) {
      publishChassisCommand(0.0, 0.0);
    }

    autoware_vehicle_msgs::msg::SteeringReport steering;
    steering.stamp = current_time;
    steering.steering_tire_angle = latest_steering_angle_;
    steering_report_pub_->publish(steering);

    autoware_vehicle_msgs::msg::ControlModeReport mode;
    mode.stamp = current_time;
    mode.mode = current_control_mode_;
    control_mode_pub_->publish(mode);

    // Periodically ensure chassis is in DRIVE/start state
    if (current_gear_ == autoware_vehicle_msgs::msg::GearCommand::DRIVE) {
      std_msgs::msg::String run_msg;
      run_msg.data = "start";
      run_static_pub_->publish(run_msg);
    }

    publishGearReport();
  }

  void publishChassisCommand(const double speed, const double steer)
  {
    if (publish_twist_cmd_) {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = now();
      cmd.header.frame_id = "base_link";
      cmd.twist.linear.x = speed;
      cmd.twist.angular.z = steer;
      twist_cmd_pub_->publish(cmd);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = steer;
    cmd_vel_pub_->publish(cmd);
  }

  void publishGearReport()
  {
    autoware_vehicle_msgs::msg::GearReport report;
    report.stamp = now();
    report.report = current_gear_;
    gear_report_pub_->publish(report);
  }

  bool isAutonomousControlMode() const
  {
    return
      current_control_mode_ == autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS ||
      current_control_mode_ ==
        autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY ||
      current_control_mode_ ==
        autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;
  }

  void onControlModeRequest(
    const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
    autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
  {
    switch (request->mode) {
      case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS:
      case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_STEER_ONLY:
      case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY:
      case autoware_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL:
        current_control_mode_ = request->mode;
        if (!isAutonomousControlMode()) {
          publishChassisCommand(0.0, 0.0);
        }
        last_control_time_ = now();
        response->success = true;
        RCLCPP_INFO(get_logger(), "Control mode changed to %u", current_control_mode_);
        return;
      case autoware_vehicle_msgs::srv::ControlModeCommand::Request::NO_COMMAND:
        response->success = true;
        return;
      default:
        response->success = false;
        RCLCPP_WARN(get_logger(), "Unsupported control mode request: %u", request->mode);
        return;
    }
  }

  double max_speed_{1.0};
  double max_steering_angle_{0.6109};
  double command_timeout_sec_{0.5};
  bool publish_twist_cmd_{true};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string twist_cmd_topic_{"/twist_cmd"};

  uint8_t current_gear_{autoware_vehicle_msgs::msg::GearCommand::DRIVE};
  uint8_t current_control_mode_{autoware_vehicle_msgs::msg::ControlModeReport::MANUAL};
  rclcpp::Time last_control_time_;
  double latest_velocity_{0.0};
  double latest_lateral_velocity_{0.0};
  double latest_heading_rate_{0.0};
  double latest_steering_angle_{0.0};

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<wtb_car_driver::msg::CarMsg>::SharedPtr car_status_sub_;
  rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_srv_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr run_static_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub_;

  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterface>());
  rclcpp::shutdown();
  return 0;
}
