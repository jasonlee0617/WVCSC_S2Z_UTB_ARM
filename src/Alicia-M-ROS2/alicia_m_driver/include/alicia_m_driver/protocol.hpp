#ifndef ALICIA_M_DRIVER__PROTOCOL_HPP_
#define ALICIA_M_DRIVER__PROTOCOL_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include <optional>

namespace alicia_m_driver
{

// ======================== 协议常量 ========================

constexpr uint8_t FRAME_HEADER = 0xAA;
constexpr uint8_t FRAME_FOOTER = 0xFF;
constexpr size_t  FRAME_OVERHEAD = 6;  // 帧头 + CMD + FUNC + LEN + CRC + 帧尾

// 指令 ID
constexpr uint8_t CMD_VERSION     = 0x01;
constexpr uint8_t CMD_TORQUE      = 0x05;
constexpr uint8_t CMD_JOINT       = 0x06;
constexpr uint8_t CMD_ENABLE      = 0x09;
constexpr uint8_t CMD_MOTOR_PARAM = 0x11;
constexpr uint8_t CMD_ERROR       = 0xEE;

// 控制目标（功能码低 7 位）
constexpr uint8_t AIM_TEACH     = 0x01;
constexpr uint8_t AIM_OPERATION = 0x02;

// 写入标志（功能码最高位）
constexpr uint8_t FUNC_WRITE_BIT = 0x80;

// 电机参数地址
constexpr uint8_t MOTOR_PARAM_CONTROL_MODE = 0x0B;

// 控制模式值（uint32_t 小端序）
constexpr uint32_t MODE_MIT = 0x01;
constexpr uint32_t MODE_PV  = 0x02;

// 数据映射范围
constexpr double POS_MIN = -12.5;  // rad
constexpr double POS_MAX =  12.5;  // rad
constexpr int    POS_BITS = 16;    // 16-bit -> [0, 65535]

constexpr double SPEED_MIN = -10.0;  // rad/s
constexpr double SPEED_MAX =  10.0;  // rad/s
constexpr int    SPEED_BITS = 12;    // 12-bit -> [0, 4095]

constexpr double TORQUE_MIN = -12.0;  // N·m
constexpr double TORQUE_MAX =  12.0;  // N·m
constexpr int    TORQUE_BITS = 12;    // 12-bit -> [0, 4095]

constexpr double KP_MIN = 0.0;
constexpr double KP_MAX = 500.0;
constexpr int    KP_BITS = 16;    // 16-bit -> [0, 65535]

constexpr double KD_MIN = 0.0;
constexpr double KD_MAX = 5.0;
constexpr int    KD_BITS = 16;    // 16-bit -> [0, 65535]

// MIT 模式默认 PID 参数 (与 SDK 一致)
constexpr double MIT_KP_LARGE = 50.0;   // 关节 1-3
constexpr double MIT_KD_LARGE = 2.0;
constexpr double MIT_KP_SMALL = 20.0;   // 关节 4-6 + 夹爪
constexpr double MIT_KD_SMALL = 1.0;

constexpr size_t NUM_JOINTS = 6;
constexpr size_t NUM_MOTORS = 7;  // 6 关节 + 1 夹爪

// ======================== 解析结果结构体 ========================

struct JointFeedback
{
  double positions[NUM_JOINTS];   // 关节位置 (rad)
  double gripper_position;        // 夹爪位置 (0-100%)
  uint16_t gripper_raw;           // 夹爪原始 16-bit 值 (调试用)
  uint8_t run_status;             // 运行状态字节
  bool valid;
};

// ======================== 工具函数 ========================

/// CRC32 校验，取低 8 位（与 SDK PyCRC.CRC32 一致）
uint8_t calculate_checksum(const uint8_t* data, size_t len);

/// 浮点数 -> 无符号整数线性映射
uint16_t float_to_uint(double x, double x_min, double x_max, int bits);

/// 无符号整数 -> 浮点数线性映射
double uint_to_float(uint16_t x_int, double x_min, double x_max, int bits);

/// 弧度 -> 16-bit 硬件值
uint16_t rad_to_hw(double rad);

/// 16-bit 硬件值 -> 弧度
double hw_to_rad(uint16_t hw);

/// 速度(rad/s) -> 12-bit 硬件值
uint16_t speed_to_hw(double speed_rad_s);

/// 扭矩(N·m) -> 12-bit 硬件值
uint16_t torque_to_hw(double torque_nm);

/// Kp -> 16-bit 硬件值
uint16_t kp_to_hw(double kp);

/// Kd -> 16-bit 硬件值
uint16_t kd_to_hw(double kd);

// ======================== 帧构建函数 ========================

/// 构建查询关节数据帧 (CMD 0x06, 读取操作臂位置)
std::vector<uint8_t> build_joint_query_frame(uint8_t aim = AIM_OPERATION);

/// 构建 PV 模式控制帧 (CMD 0x06, 写入位置+速度)
/// positions: 6 个关节角度 (rad), speeds: 6 个关节速度 (rad/s)
/// gripper_hw: 夹爪硬件值, gripper_speed_hw: 夹爪速度硬件值
std::vector<uint8_t> build_pv_control_frame(
  const double positions[NUM_JOINTS],
  const double speeds[NUM_JOINTS],
  uint16_t gripper_hw,
  uint16_t gripper_speed_hw,
  uint8_t aim = AIM_OPERATION);

/// 构建 MIT 模式初始化帧 (CMD 0x06, 写入 pos+speed+torque+kp+kd)
/// 设置 MIT PID 参数并将电机切入 MIT 控制状态 (与 SDK MIT_INIT_COMMAND 等效)
/// kp: 位置环比例增益 [0, 500], kd: 速度环阻尼增益 [0, 5]
std::vector<uint8_t> build_mit_init_frame(
  const double positions[NUM_JOINTS],
  uint16_t gripper_hw,
  double kp, double kd,
  uint8_t aim = AIM_OPERATION);

/// 构建 MIT 模式控制帧 (CMD 0x06, 仅写入位置, 每电机 2 字节)
std::vector<uint8_t> build_mit_control_frame(
  const double positions[NUM_JOINTS],
  uint16_t gripper_hw,
  uint8_t aim = AIM_OPERATION);

/// 构建电机模式切换帧 (CMD 0x11, addr 0x0B)
std::vector<uint8_t> build_mode_switch_frame(
  uint32_t mode, uint8_t aim = AIM_OPERATION);

/// 构建使能/失能帧 (CMD 0x09)
std::vector<uint8_t> build_enable_frame(
  bool enable, uint8_t aim = AIM_OPERATION);

/// 构建力矩锁定帧 (CMD 0x05)
std::vector<uint8_t> build_torque_frame(
  bool lock, uint8_t aim = AIM_OPERATION);

// ======================== 帧解析函数 ========================

/// 从接收缓冲区中提取一帧完整数据，返回帧长度，0 表示无完整帧
size_t extract_frame(const uint8_t* buf, size_t buf_len,
                     std::vector<uint8_t>& frame_out);

/// 解析关节反馈帧 (CMD 0x06 的请求响应)
std::optional<JointFeedback> parse_joint_feedback(
  const std::vector<uint8_t>& frame);

}  // namespace alicia_m_driver

#endif  // ALICIA_M_DRIVER__PROTOCOL_HPP_
