# 感知管道发布的规范二维病果目标消息，由 SprayTask/视觉伺服节点消费。
# 它携带全图像素坐标，内部 ROI 处理后的结果已映射回原图。
# 目标账目通过 mission/tree/target ID 避免重复喷洒已处理的目标。
std_msgs/Header header                 # 源图像帧 ID 和检测时间戳。
string mission_id                      # 当前 ExecuteSpray 任务标识符。
string tree_id                         # 当前植物标识符。
string target_id                       # 稳定的跟踪器/账目标识符。
bool valid                             # False 表示当前没有可用的控制目标。
float32 confidence                     # 按模型约定的置信度，范围 [0, 1]。
# 全图像素中的控制点：分割后端使用掩膜安全点；
# 检测后端无掩膜，因此使用检测框中心。
float32 center_u
float32 center_v
float32 width                          # 检测框宽度（全图像素）。
float32 height                         # 检测框高度（全图像素）。
uint32 image_width                     # 源图像宽度（像素）。
uint32 image_height                    # 源图像高度（像素）。