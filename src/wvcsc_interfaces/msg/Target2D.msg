# 感知流水线发布的规范二维病态目标消息，由 SprayTask 和视觉伺服消费。
# 坐标始终属于完整相机图像；当前感知默认直接处理整幅图像，不依赖树 ROI。
# 目标账本以 mission_id 与 target_id 避免重复喷洒；路线 Point ID 不参与视觉目标匹配。
std_msgs/Header header                 # 源相机图像的 frame_id 和检测时间戳。
string mission_id                      # 当前 ExecuteSpray 任务关联 ID。
# 当前感知会话内唯一且稳定的目标追踪 ID；实现应使用 UUIDv4。它不是跨任务/重启永久的物理叶片 ID。
string target_id
bool valid                             # False 表示当前没有可供闭环控制的目标；其余几何字段不得被当作有效目标使用。
float32 confidence                     # 模型输出置信度，按后端约定应位于 [0, 1]。
# 完整图像中的控制点：segment 后端为掩膜安全点；detect 后端无掩膜，必须为检测框中心。
float32 center_u
float32 center_v
float32 width                          # 对应检测框的完整图像宽度（像素）。
float32 height                         # 对应检测框的完整图像高度（像素）。
uint32 image_width                     # 源完整图像宽度（像素）。
uint32 image_height                    # 源完整图像高度（像素）。
