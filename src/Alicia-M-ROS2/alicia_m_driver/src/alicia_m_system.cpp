#include "alicia_m_driver/alicia_m_system.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alicia_m_driver
{

static const rclcpp::Logger kLogger = rclcpp::get_logger("AliciaHardwareInterface");

// 夹爪百分比 <-> 硬件值映射 (与 SDK 一致)
static constexpr uint16_t GRIP_CLOSE_HW = 32768;  // 0%
static constexpr uint16_t GRIP_OPEN_HW  = 39688;  // 100%

// left_finger 关节的行程范围 (URDF 中定义: [-0.05, 0])
static constexpr double FINGER_POS_MIN = -0.05;
static constexpr double FINGER_POS_MAX =  0.0;

static uint16_t gripper_percent_to_hw(double pct)
{
  pct = std::max(0.0, std::min(100.0, pct));
  return static_cast<uint16_t>(
    GRIP_CLOSE_HW + (pct / 100.0) * (GRIP_OPEN_HW - GRIP_CLOSE_HW));
}

// left_finger 位置 (m) -> 夹爪百分比 (0-100)
// URDF 几何: 0 m = 全开 (95mm 指距), -0.05 m = 全关 (5mm 指距)
// 硬件: 0% = 闭合 (32768), 100% = 张开 (39688)
static double finger_pos_to_percent(double pos)
{
  double pct = (pos - FINGER_POS_MIN) / (FINGER_POS_MAX - FINGER_POS_MIN) * 100.0;
  return std::max(0.0, std::min(100.0, pct));
}

// 夹爪百分比 -> left_finger 位置 (m)
static double percent_to_finger_pos(double pct)
{
  pct = std::max(0.0, std::min(100.0, pct));
  return FINGER_POS_MIN + (pct / 100.0) * (FINGER_POS_MAX - FINGER_POS_MIN);
}

// ======================== 生命周期回调 ========================

hardware_interface::CallbackReturn AliciaHardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 读取硬件参数
  serial_port_name_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
  baudrate_ = info_.hardware_parameters.count("baudrate")
    ? std::stoi(info_.hardware_parameters.at("baudrate")) : 1000000;
  control_mode_ = info_.hardware_parameters.count("control_mode")
    ? info_.hardware_parameters.at("control_mode") : "pv";
  default_speed_ = info_.hardware_parameters.count("default_speed")
    ? std::stod(info_.hardware_parameters.at("default_speed")) : 1.0;
  mit_kp_ = info_.hardware_parameters.count("mit_kp")
    ? std::stod(info_.hardware_parameters.at("mit_kp")) : 50.0;
  mit_kd_ = info_.hardware_parameters.count("mit_kd")
    ? std::stod(info_.hardware_parameters.at("mit_kd")) : 2.0;

  control_aim_ = AIM_OPERATION;

  // 初始化关节存储（6 个旋转关节）
  hw_positions_.resize(NUM_JOINTS, 0.0);
  hw_velocities_.resize(NUM_JOINTS, 0.0);
  hw_commands_.resize(NUM_JOINTS, 0.0);
  prev_hw_positions_.resize(NUM_JOINTS, 0.0);

  // 夹爪初始化
  gripper_position_ = 0.0;
  gripper_velocity_ = 0.0;
  gripper_command_  = 0.0;
  prev_gripper_position_ = 0.0;

  last_gripper_raw_ = 0;
  gripper_feedback_stale_cycles_ = 0;
  gripper_echo_active_ = false;

  first_feedback_received_ = false;

  RCLCPP_INFO(kLogger,
    "Initialized: port=%s, baud=%d, mode=%s",
    serial_port_name_.c_str(), baudrate_, control_mode_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!serial_.open(serial_port_name_, baudrate_)) {
    RCLCPP_ERROR(kLogger, "Failed to open serial port %s", serial_port_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_.flush_input();
  rx_accum_.clear();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(kLogger, "Activating hardware...");

  for (int attempt = 0; attempt < 20; ++attempt) {
    auto query = build_joint_query_frame(control_aim_);
    serial_.write(query);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    poll_serial();
    if (try_parse_feedback()) {
      first_feedback_received_ = true;
      break;
    }
  }

  if (first_feedback_received_) {
    RCLCPP_INFO(kLogger, "\033[1;32m\nJoint positions (deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\033[0m",
      hw_positions_[0] * 180.0 / M_PI,
      hw_positions_[1] * 180.0 / M_PI,
      hw_positions_[2] * 180.0 / M_PI,
      hw_positions_[3] * 180.0 / M_PI,
      hw_positions_[4] * 180.0 / M_PI,
      hw_positions_[5] * 180.0 / M_PI);
    RCLCPP_INFO(kLogger, "\033[1;32m\nGripper position: %.1f%%\033[0m",
      finger_pos_to_percent(gripper_position_));
  } else {
    RCLCPP_WARN(kLogger, "No initial joint feedback, using default zero position");
  }

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_commands_[i] = hw_positions_[i];
    prev_hw_positions_[i] = hw_positions_[i];
  }
  gripper_command_ = gripper_position_;
  prev_gripper_position_ = gripper_position_;

  double grip_pct = finger_pos_to_percent(gripper_command_);
  uint16_t grip_hw = gripper_percent_to_hw(grip_pct);

  if (control_mode_ == "mit") {
    auto mit_init = build_mit_init_frame(
      hw_commands_.data(), grip_hw, mit_kp_, mit_kd_, control_aim_);
    serial_.write(mit_init);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    serial_.write(mit_init);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto torque_frame = build_torque_frame(true, control_aim_);
    serial_.write(torque_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto frame = build_mit_control_frame(
      hw_commands_.data(), grip_hw, control_aim_);
    serial_.write(frame);
  } else {
    auto enable_frame = build_enable_frame(true, control_aim_);
    serial_.write(enable_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    double speeds[NUM_JOINTS];
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      speeds[i] = default_speed_;
    }
    uint16_t grip_speed_hw = speed_to_hw(1.0);
    auto frame = build_pv_control_frame(
      hw_commands_.data(), speeds, grip_hw, grip_speed_hw, control_aim_);
    serial_.write(frame);
  }

  RCLCPP_INFO(kLogger, "\033[1;32m\nHardware activated (%s mode)\033[0m", control_mode_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(kLogger, "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(kLogger, "Cleaning up, closing serial port");

  if (serial_.is_open()) {
    auto disable_frame = build_enable_frame(false, control_aim_);
    serial_.write(disable_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  serial_.close();
  rx_accum_.clear();
  first_feedback_received_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ======================== 接口导出 ========================

std::vector<hardware_interface::StateInterface>
AliciaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];

    if (joint.name == "left_finger") {
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &gripper_position_);
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_);
    } else {
      // 按 URDF 中关节顺序找到对应索引
      // joint1..joint6 对应 hw_positions_[0..5]
      size_t idx = 0;
      for (size_t j = 0; j < NUM_JOINTS; ++j) {
        if (joint.name == "joint" + std::to_string(j + 1)) {
          idx = j;
          break;
        }
      }
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_positions_[idx]);
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[idx]);
    }
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AliciaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];

    if (joint.name == "left_finger") {
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &gripper_command_);
    } else {
      size_t idx = 0;
      for (size_t j = 0; j < NUM_JOINTS; ++j) {
        if (joint.name == "joint" + std::to_string(j + 1)) {
          idx = j;
          break;
        }
      }
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_[idx]);
    }
  }

  return interfaces;
}

// ======================== 读写周期 ========================

hardware_interface::return_type AliciaHardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (!serial_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  // 发送查询帧
  auto query = build_joint_query_frame(control_aim_);
  serial_.write(query);

  // 非阻塞地读取串口数据 (两次尝试降低延迟)
  poll_serial();
  poll_serial();

  // 尝试解析反馈，收到新数据时估算速度
  if (try_parse_feedback()) {
    double dt = period.seconds();
    if (dt > 0.0) {
      for (size_t i = 0; i < NUM_JOINTS; ++i) {
        hw_velocities_[i] = (hw_positions_[i] - prev_hw_positions_[i]) / dt;
        prev_hw_positions_[i] = hw_positions_[i];
      }

      // 夹爪: 当硬件反馈不更新时使用插值追踪命令位置
      if (gripper_echo_active_) {
        double diff = gripper_command_ - gripper_position_;
        double max_delta = GRIPPER_INTERP_SPEED * dt;
        if (std::abs(diff) <= max_delta) {
          gripper_position_ = gripper_command_;
        } else {
          gripper_position_ += std::copysign(max_delta, diff);
        }
      }

      gripper_velocity_ = (gripper_position_ - prev_gripper_position_) / dt;
      prev_gripper_position_ = gripper_position_;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AliciaHardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!serial_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  double grip_pct = finger_pos_to_percent(gripper_command_);
  uint16_t grip_hw = gripper_percent_to_hw(grip_pct);

  if (control_mode_ == "mit") {
    auto frame = build_mit_control_frame(
      hw_commands_.data(), grip_hw, control_aim_);
    serial_.write(frame);
  } else {
    double speeds[NUM_JOINTS];
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      speeds[i] = default_speed_;
    }
    uint16_t grip_speed_hw = speed_to_hw(1.0);
    auto frame = build_pv_control_frame(
      hw_commands_.data(), speeds, grip_hw, grip_speed_hw, control_aim_);
    serial_.write(frame);
  }

  return hardware_interface::return_type::OK;
}

// ======================== 私有辅助方法 ========================

bool AliciaHardwareInterface::send_and_wait(
  const std::vector<uint8_t>& frame, int retries, int wait_ms)
{
  for (int i = 0; i < retries; ++i) {
    serial_.write(frame);

    // 等待响应
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      uint8_t buf[64];
      ssize_t n = serial_.read(buf, sizeof(buf), 5);
      if (n > 0) {
        // 收到任何响应即认为成功（配置阶段的简化处理）
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return false;
}

void AliciaHardwareInterface::poll_serial()
{
  ssize_t n = serial_.read(rx_buf_, RX_BUF_SIZE, 3);
  if (n > 0) {
    rx_accum_.insert(rx_accum_.end(), rx_buf_, rx_buf_ + n);
  }

  // 防止缓冲区无限增长
  if (rx_accum_.size() > 512) {
    rx_accum_.erase(rx_accum_.begin(),
                    rx_accum_.begin() + static_cast<long>(rx_accum_.size() - 256));
  }
}

bool AliciaHardwareInterface::try_parse_feedback()
{
  if (rx_accum_.empty()) return false;

  bool got_joint_data = false;

  while (!rx_accum_.empty()) {
    std::vector<uint8_t> frame;
    size_t consumed = extract_frame(rx_accum_.data(), rx_accum_.size(), frame);
    if (consumed == 0) break;

    rx_accum_.erase(rx_accum_.begin(),
                    rx_accum_.begin() + static_cast<long>(consumed));

    auto fb = parse_joint_feedback(frame);
    if (!fb.has_value()) continue;

    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      hw_positions_[i] = fb->positions[i];
    }

    if (fb->gripper_raw != last_gripper_raw_) {
      last_gripper_raw_ = fb->gripper_raw;
      gripper_feedback_stale_cycles_ = 0;
      if (gripper_echo_active_) {
        gripper_echo_active_ = false;
      }
      gripper_position_ = percent_to_finger_pos(fb->gripper_position);
    } else {
      double cmd_diff = std::abs(gripper_command_ - gripper_position_);
      if (cmd_diff > 0.005) {
        gripper_feedback_stale_cycles_++;
        if (!gripper_echo_active_ &&
            gripper_feedback_stale_cycles_ >= GRIPPER_STALE_THRESHOLD) {
          gripper_echo_active_ = true;
        }
      } else {
        gripper_feedback_stale_cycles_ = 0;
      }
      if (!gripper_echo_active_) {
        gripper_position_ = percent_to_finger_pos(fb->gripper_position);
      }
    }
    got_joint_data = true;
  }

  return got_joint_data;
}

}  // namespace alicia_m_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  alicia_m_driver::AliciaHardwareInterface,
  hardware_interface::SystemInterface)
