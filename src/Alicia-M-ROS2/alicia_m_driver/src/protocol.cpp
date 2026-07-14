#include "alicia_m_driver/protocol.hpp"
#include <cstring>
#include <algorithm>

namespace alicia_m_driver
{

// ======================== CRC32 实现 ========================
// 使用标准 CRC-32 多项式 (0xEDB88320, 反射版)
// 与 SDK 中 PyCRC.CRC32 一致，最终取低 8 位

static uint32_t crc32_table[256];
static bool crc32_table_initialized = false;

static void init_crc32_table()
{
  if (crc32_table_initialized) return;
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t crc = i;
    for (int j = 0; j < 8; ++j) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0u);
    }
    crc32_table[i] = crc;
  }
  crc32_table_initialized = true;
}

static uint32_t crc32_compute(const uint8_t* data, size_t len)
{
  init_crc32_table();
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFF];
  }
  return crc ^ 0xFFFFFFFFu;
}

uint8_t calculate_checksum(const uint8_t* data, size_t len)
{
  return static_cast<uint8_t>(crc32_compute(data, len) & 0xFF);
}

// ======================== 数据映射 ========================

uint16_t float_to_uint(double x, double x_min, double x_max, int bits)
{
  double span = x_max - x_min;
  if (span <= 0.0) return 0;
  uint32_t max_val = (1u << bits) - 1;
  x = std::max(x_min, std::min(x_max, x));
  auto val = static_cast<uint32_t>((x - x_min) * static_cast<double>(max_val) / span);
  return static_cast<uint16_t>(std::min(val, max_val));
}

double uint_to_float(uint16_t x_int, double x_min, double x_max, int bits)
{
  uint32_t max_val = (1u << bits) - 1;
  if (x_int > max_val) x_int = static_cast<uint16_t>(max_val);
  double span = x_max - x_min;
  return static_cast<double>(x_int) * span / static_cast<double>(max_val) + x_min;
}

uint16_t rad_to_hw(double rad)
{
  return float_to_uint(rad, POS_MIN, POS_MAX, POS_BITS);
}

double hw_to_rad(uint16_t hw)
{
  return uint_to_float(hw, POS_MIN, POS_MAX, POS_BITS);
}

uint16_t speed_to_hw(double speed_rad_s)
{
  return float_to_uint(speed_rad_s, SPEED_MIN, SPEED_MAX, SPEED_BITS);
}

uint16_t torque_to_hw(double torque_nm)
{
  return float_to_uint(torque_nm, TORQUE_MIN, TORQUE_MAX, TORQUE_BITS);
}

uint16_t kp_to_hw(double kp)
{
  return float_to_uint(kp, KP_MIN, KP_MAX, KP_BITS);
}

uint16_t kd_to_hw(double kd)
{
  return float_to_uint(kd, KD_MIN, KD_MAX, KD_BITS);
}

// ======================== 帧校验辅助 ========================

// 向帧末尾追加校验位和帧尾，帧的 [1..-2) 区间用于计算校验
static void finalize_frame(std::vector<uint8_t>& frame)
{
  // CRC 计算范围: 帧头之后、校验位之前 (frame[1] .. frame[size-3])
  uint8_t crc = calculate_checksum(frame.data() + 1, frame.size() - 3);
  frame[frame.size() - 2] = crc;
  frame[frame.size() - 1] = FRAME_FOOTER;
}

// ======================== 帧构建 ========================

std::vector<uint8_t> build_joint_query_frame(uint8_t aim)
{
  // [AA] [06] [aim] [02] [00] [01] [CRC] [FF]
  std::vector<uint8_t> frame = {
    FRAME_HEADER, CMD_JOINT, aim, 0x02, 0x00, 0x01, 0x00, FRAME_FOOTER
  };
  finalize_frame(frame);
  return frame;
}

std::vector<uint8_t> build_pv_control_frame(
  const double positions[NUM_JOINTS],
  const double speeds[NUM_JOINTS],
  uint16_t gripper_hw,
  uint16_t gripper_speed_hw,
  uint8_t aim)
{
  // PV 模式: 每电机 4 字节 (2B 位置 + 2B 速度), 7 个电机 = 28 字节
  // 前缀 2 字节: [0x00(写入+位置模式), 0x02(每电机4/2=2)]
  constexpr size_t PREFIX_LEN = 2;
  constexpr size_t MOTOR_DATA_LEN = NUM_MOTORS * 4;  // 7 * 4 = 28
  constexpr size_t DATA_LEN = PREFIX_LEN + MOTOR_DATA_LEN;
  constexpr size_t FRAME_SIZE = FRAME_OVERHEAD + DATA_LEN;

  std::vector<uint8_t> frame(FRAME_SIZE, 0);
  frame[0] = FRAME_HEADER;
  frame[1] = CMD_JOINT;
  frame[2] = FUNC_WRITE_BIT | aim;  // 0x82 (操作臂写入)
  frame[3] = static_cast<uint8_t>(DATA_LEN);
  frame[4] = 0x00;  // 写入 + 位置/速度模式
  frame[5] = 0x02;  // 每电机 4 字节 (4/2=2)

  size_t offset = 6;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    uint16_t pos_hw = rad_to_hw(positions[i]);
    uint16_t spd_hw = speed_to_hw(speeds[i]);
    frame[offset++] = pos_hw & 0xFF;
    frame[offset++] = (pos_hw >> 8) & 0xFF;
    frame[offset++] = spd_hw & 0xFF;
    frame[offset++] = (spd_hw >> 8) & 0xFF;
  }

  // 夹爪
  frame[offset++] = gripper_hw & 0xFF;
  frame[offset++] = (gripper_hw >> 8) & 0xFF;
  frame[offset++] = gripper_speed_hw & 0xFF;
  frame[offset++] = (gripper_speed_hw >> 8) & 0xFF;

  finalize_frame(frame);
  return frame;
}

std::vector<uint8_t> build_mit_init_frame(
  const double positions[NUM_JOINTS],
  uint16_t gripper_hw,
  double kp, double kd,
  uint8_t aim)
{
  // MIT 初始化: 每电机 10 字节 (pos:2B + speed:2B + torque:2B + kp:2B + kd:2B)
  // 前缀 2 字节: [0x00(起始addr=位置), 0x05(偏移数量=5个参数)]
  // 与 SDK MIT_INIT_COMMAND 等效，用于配置 MIT PID 参数并进入 MIT 控制状态
  constexpr size_t PREFIX_LEN = 2;
  constexpr size_t BYTES_PER_MOTOR = 10;
  constexpr size_t MOTOR_DATA_LEN = NUM_MOTORS * BYTES_PER_MOTOR;  // 7 * 10 = 70
  constexpr size_t DATA_LEN = PREFIX_LEN + MOTOR_DATA_LEN;         // 72 = 0x48
  constexpr size_t FRAME_SIZE = FRAME_OVERHEAD + DATA_LEN;

  std::vector<uint8_t> frame(FRAME_SIZE, 0);
  frame[0] = FRAME_HEADER;
  frame[1] = CMD_JOINT;
  frame[2] = FUNC_WRITE_BIT | aim;
  frame[3] = static_cast<uint8_t>(DATA_LEN);
  frame[4] = 0x00;  // 起始地址: 位置
  frame[5] = 0x05;  // 偏移数量: pos+speed+torque+kp+kd

  uint16_t speed_center = speed_to_hw(0.0);
  uint16_t torque_center = torque_to_hw(0.0);
  uint16_t kp_hw_val = kp_to_hw(kp);
  uint16_t kd_hw_val = kd_to_hw(kd);

  size_t offset = 6;
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    uint16_t pos_hw;
    if (i < NUM_JOINTS) {
      pos_hw = rad_to_hw(positions[i]);
    } else {
      pos_hw = gripper_hw;
    }
    frame[offset++] = pos_hw & 0xFF;
    frame[offset++] = (pos_hw >> 8) & 0xFF;

    frame[offset++] = speed_center & 0xFF;
    frame[offset++] = (speed_center >> 8) & 0xFF;

    frame[offset++] = torque_center & 0xFF;
    frame[offset++] = (torque_center >> 8) & 0xFF;

    frame[offset++] = kp_hw_val & 0xFF;
    frame[offset++] = (kp_hw_val >> 8) & 0xFF;
    frame[offset++] = kd_hw_val & 0xFF;
    frame[offset++] = (kd_hw_val >> 8) & 0xFF;
  }

  finalize_frame(frame);
  return frame;
}

std::vector<uint8_t> build_mit_control_frame(
  const double positions[NUM_JOINTS],
  uint16_t gripper_hw,
  uint8_t aim)
{
  // MIT 位置控制: 每电机 2 字节 (仅位置), 7 个电机 = 14 字节
  // 前缀 2 字节: [0x00(起始addr=位置), 0x01(偏移数量=1)]
  // 对应 SDK PATTERN_MIT_POSITION = (0x00, 0x01)
  constexpr size_t PREFIX_LEN = 2;
  constexpr size_t MOTOR_DATA_LEN = NUM_MOTORS * 2;  // 7 * 2 = 14
  constexpr size_t DATA_LEN = PREFIX_LEN + MOTOR_DATA_LEN;
  constexpr size_t FRAME_SIZE = FRAME_OVERHEAD + DATA_LEN;

  std::vector<uint8_t> frame(FRAME_SIZE, 0);
  frame[0] = FRAME_HEADER;
  frame[1] = CMD_JOINT;
  frame[2] = FUNC_WRITE_BIT | aim;
  frame[3] = static_cast<uint8_t>(DATA_LEN);
  frame[4] = 0x00;  // 起始地址: 位置
  frame[5] = 0x01;  // 偏移数量: 仅位置

  size_t offset = 6;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    uint16_t pos_hw = rad_to_hw(positions[i]);
    frame[offset++] = pos_hw & 0xFF;
    frame[offset++] = (pos_hw >> 8) & 0xFF;
  }

  // 夹爪
  frame[offset++] = gripper_hw & 0xFF;
  frame[offset++] = (gripper_hw >> 8) & 0xFF;

  finalize_frame(frame);
  return frame;
}

std::vector<uint8_t> build_mode_switch_frame(uint32_t mode, uint8_t aim)
{
  // CMD 0x11, 功能码 = 0x80 | aim
  // 数据: [起始电机ID=0x01] [电机数量=0x07] [参数地址=0x0B] [模式值 LE32 4字节]
  // 数据长度 = 7
  std::vector<uint8_t> frame = {
    FRAME_HEADER,
    CMD_MOTOR_PARAM,
    static_cast<uint8_t>(FUNC_WRITE_BIT | aim),
    0x07,  // 数据长度
    0x01,  // 起始电机 ID 索引 (从 1 开始)
    0x07,  // 偏移电机数量 (7 个)
    MOTOR_PARAM_CONTROL_MODE,  // 参数地址 0x0B
    static_cast<uint8_t>(mode & 0xFF),
    static_cast<uint8_t>((mode >> 8) & 0xFF),
    static_cast<uint8_t>((mode >> 16) & 0xFF),
    static_cast<uint8_t>((mode >> 24) & 0xFF),
    0x00,  // CRC 占位
    FRAME_FOOTER
  };
  finalize_frame(frame);
  return frame;
}

std::vector<uint8_t> build_enable_frame(bool enable, uint8_t aim)
{
  // CMD 0x09, 功能码 = 0x80 | aim, 数据长度=1, 数据=0x01(使能)/0x00(失能)
  std::vector<uint8_t> frame = {
    FRAME_HEADER,
    CMD_ENABLE,
    static_cast<uint8_t>(FUNC_WRITE_BIT | aim),
    0x01,
    static_cast<uint8_t>(enable ? 0x01 : 0x00),
    0x00,  // CRC 占位
    FRAME_FOOTER
  };
  finalize_frame(frame);
  return frame;
}

std::vector<uint8_t> build_torque_frame(bool lock, uint8_t aim)
{
  // CMD 0x05, 功能码 = aim (不含写入位，因为力矩指令格式不同)
  // 数据: [起始关节ID=0x01] [偏移数量=0x07]
  // lock=true 表示锁定力矩（上电），lock=false 表示卸力
  (void)lock;  // 力矩锁定/解锁通过 enable 帧控制
  std::vector<uint8_t> frame = {
    FRAME_HEADER,
    CMD_TORQUE,
    aim,
    0x02,  // 数据长度
    0x01,  // 起始关节
    0x07,  // 偏移数量
    0x00,  // CRC 占位
    FRAME_FOOTER
  };
  finalize_frame(frame);
  return frame;
}

// ======================== 帧解析 ========================

size_t extract_frame(const uint8_t* buf, size_t buf_len,
                     std::vector<uint8_t>& frame_out)
{
  frame_out.clear();
  if (buf_len < FRAME_OVERHEAD) return 0;

  for (size_t i = 0; i < buf_len; ++i) {
    if (buf[i] != FRAME_HEADER) continue;

    // 检查是否有足够数据读取 LEN 字段
    if (i + 3 >= buf_len) return 0;

    size_t data_len = buf[i + 3];
    size_t frame_len = data_len + FRAME_OVERHEAD;

    if (i + frame_len > buf_len) return 0;

    // 检查帧尾
    if (buf[i + frame_len - 1] != FRAME_FOOTER) continue;

    // 验证 CRC: 计算范围 [CMD .. 最后一个数据字节]
    uint8_t expected_crc = calculate_checksum(
      buf + i + 1, frame_len - 3);  // 跳过帧头，不含 CRC 和帧尾
    uint8_t received_crc = buf[i + frame_len - 2];

    if (expected_crc != received_crc) continue;

    frame_out.assign(buf + i, buf + i + frame_len);
    return i + frame_len;  // 返回已消耗的总字节数
  }
  return 0;
}

std::optional<JointFeedback> parse_joint_feedback(
  const std::vector<uint8_t>& frame)
{
  // 最小帧长度: 头(1) + CMD(1) + FUNC(1) + LEN(1) + 数据类型(1) + 偏移(1)
  //             + 7电机*2字节(14) + 状态(1) + CRC(1) + 尾(1) = 23
  if (frame.size() < 23) return std::nullopt;
  if (frame[1] != CMD_JOINT) return std::nullopt;

  uint8_t func = frame[2];
  // 检查是否为请求响应（非写入响应）: 高位为 0
  if ((func & FUNC_WRITE_BIT) != 0) return std::nullopt;

  size_t data_len = frame[3];
  if (frame.size() < FRAME_OVERHEAD + data_len) return std::nullopt;

  // data_bytes 从 frame[4] 开始
  const uint8_t* data = frame.data() + 4;

  // 数据类型标志
  uint8_t data_type = data[0];
  bool is_feedback = (data_type & 0x80) != 0;
  if (!is_feedback) return std::nullopt;

  // payload 从 data[2] 开始 (跳过数据类型和偏移)
  if (data_len < 2 + NUM_MOTORS * 2 + 1) return std::nullopt;

  const uint8_t* payload = data + 2;
  size_t payload_len = data_len - 2;

  JointFeedback fb{};
  fb.valid = true;

  // 解析 7 个电机的位置数据 (每个 2 字节小端序)
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    uint16_t raw = static_cast<uint16_t>(payload[i * 2]) |
                   (static_cast<uint16_t>(payload[i * 2 + 1]) << 8);
    fb.positions[i] = hw_to_rad(raw);
  }

  // 夹爪 (第 7 个电机)
  {
    uint16_t raw = static_cast<uint16_t>(payload[NUM_JOINTS * 2]) |
                   (static_cast<uint16_t>(payload[NUM_JOINTS * 2 + 1]) << 8);
    fb.gripper_raw = raw;
    constexpr uint16_t GRIP_CLOSE = 32768;
    constexpr uint16_t GRIP_OPEN  = 39688;
    double pct = static_cast<double>(raw - GRIP_CLOSE) /
                 static_cast<double>(GRIP_OPEN - GRIP_CLOSE) * 100.0;
    fb.gripper_position = std::max(0.0, std::min(100.0, pct));
  }

  // 运行状态 (payload 最后一字节)
  fb.run_status = payload[payload_len - 1];

  return fb;
}

}  // namespace alicia_m_driver
