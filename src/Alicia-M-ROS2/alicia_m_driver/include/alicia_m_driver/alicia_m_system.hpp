#ifndef ALICIA_M_DRIVER__ALICIA_M_SYSTEM_HPP_
#define ALICIA_M_DRIVER__ALICIA_M_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "alicia_m_driver/serial_port.hpp"
#include "alicia_m_driver/protocol.hpp"

namespace alicia_m_driver
{

/// Alicia-M 机械臂 ros2_control 硬件接口
///
/// 通过串口与下位机通信，支持 PV（位置+速度）和 MIT 控制模式。
/// 实现非阻塞读写，在 read()/write() 周期中完成数据交换。
class AliciaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AliciaHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // 硬件参数
  std::string serial_port_name_;
  int baudrate_;
  std::string control_mode_;  // "pv" 或 "mit"
  uint8_t control_aim_;

  // 串口
  SerialPort serial_;

  // 接收缓冲区
  static constexpr size_t RX_BUF_SIZE = 256;
  uint8_t rx_buf_[RX_BUF_SIZE];
  std::vector<uint8_t> rx_accum_;

  // 关节状态
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // 夹爪状态（left_finger 的 position 指令映射到夹爪硬件值）
  double gripper_position_;
  double gripper_velocity_;
  double gripper_command_;

  // 默认速度 (rad/s)
  double default_speed_;

  // MIT 模式 PID 参数
  double mit_kp_;  // 位置环比例增益 [0, 500]
  double mit_kd_;  // 速度环阻尼增益 [0, 5]

  // 速度估算用的上一周期位置缓存
  std::vector<double> prev_hw_positions_;
  double prev_gripper_position_;

  // 夹爪反馈质量跟踪: 硬件可能不反馈真实夹爪位置
  uint16_t last_gripper_raw_;
  int gripper_feedback_stale_cycles_;
  bool gripper_echo_active_;
  static constexpr int GRIPPER_STALE_THRESHOLD = 50;   // 0.5s @ 100Hz
  static constexpr double GRIPPER_INTERP_SPEED = 0.05; // m/s

  // 是否已收到第一帧有效反馈
  bool first_feedback_received_;

  /// 发送命令帧并等待响应（带重试），用于配置阶段
  bool send_and_wait(const std::vector<uint8_t>& frame,
                     int retries = 3, int wait_ms = 50);

  /// 非阻塞地从串口读取数据并追加到 rx_accum_
  void poll_serial();

  /// 尝试从 rx_accum_ 提取并解析关节反馈
  bool try_parse_feedback();
};

}  // namespace alicia_m_driver

#endif  // ALICIA_M_DRIVER__ALICIA_M_SYSTEM_HPP_
