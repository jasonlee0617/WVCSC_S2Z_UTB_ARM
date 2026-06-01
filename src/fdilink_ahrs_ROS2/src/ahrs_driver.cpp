#include <ahrs_driver.h>

rclcpp::Node::SharedPtr nh_=nullptr;

// FDILink命名空间，包含AHRS设备通信相关功能
namespace FDILink
{
/**
 * @brief AHRS设备驱动节点构造函数
 * 初始化ROS2节点、参数服务、串口连接和发布器
 */
ahrsBringup::ahrsBringup()
: rclcpp::Node ("ahrs_bringup")
{
  //topic_name & frame_id  加载参数服务器
  //声明和获取ROS2参数，用于配置话题名称、串口参数等
  this->declare_parameter("if_debug_",false);
  this->get_parameter("if_debug_", if_debug_);

  this->declare_parameter<std::int8_t>("device_type_",1);
  this->get_parameter("device_type_",  device_type_);

  this->declare_parameter<std::string>("imu_topic","/imu");
  this->get_parameter("imu_topic",  imu_topic);

  this->declare_parameter<std::string>("imu_frame_id_","gyro_link");
  this->get_parameter("imu_frame_id_",   imu_frame_id_);

  this->declare_parameter<std::string>("mag_pose_2d_topic","/mag_pose_2d");
  this->get_parameter("mag_pose_2d_topic", mag_pose_2d_topic);

  this->declare_parameter<std::string>("Euler_angles_topic","/euler_angles");
  this->get_parameter("Euler_angles_topic", Euler_angles_topic);

  this->declare_parameter<std::string>("gps_topic","/gps/fix");
  this->get_parameter("gps_topic", gps_topic);


  this->declare_parameter<std::string>("Magnetic_topic","/magnetic");
  this->get_parameter("Magnetic_topic", Magnetic_topic);

  this->declare_parameter<std::string>("twist_topic","/system_speed");
  this->get_parameter("twist_topic", twist_topic);

  this->declare_parameter<std::string>("NED_odom_topic","/NED_odometry");
  this->get_parameter("NED_odom_topic", NED_odom_topic);

  this->declare_parameter<std::string>("serial_port_","/dev/ttyUSB0");
  this->get_parameter("serial_port_", serial_port_);

  this->declare_parameter<std::int64_t>("serial_baud_",921600);
  this->get_parameter("serial_baud_", serial_baud_);  


  
  //publisher
  //创建各种数据话题的发布器
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic.c_str(), 10);  // IMU数据发布器
  gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic.c_str(), 10);  // GPS定位数据发布器

  mag_pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic.c_str(), 10);  // 2D姿态发布器

  Euler_angles_pub_ = create_publisher<geometry_msgs::msg::Vector3>(Euler_angles_topic.c_str(), 10);  // 欧拉角发布器
  Magnetic_pub_ = create_publisher<geometry_msgs::msg::Vector3>(Magnetic_topic.c_str(), 10);  // 磁力计数据发布器
 
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic.c_str(), 10);  // 速度话题发布器
  NED_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(NED_odom_topic.c_str(), 10);  // NED坐标系里程计发布器


  //setp up serial  设置串口参数并打开串口
  //配置并打开与AHRS设备的串口通信
  try
  {
    serial_.setPort(serial_port_);  // 设置串口设备路径
    serial_.setBaudrate(serial_baud_);  // 设置波特率
    serial_.setFlowcontrol(serial::flowcontrol_none);  // 设置流控制为无
    serial_.setParity(serial::parity_none); //default is parity_none，设置奇偶校验位
    serial_.setStopbits(serial::stopbits_one);  // 设置停止位为1位
    serial_.setBytesize(serial::eightbits);  // 设置数据位为8位
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
    serial_.setTimeout(time_out);  // 设置超时时间
    serial_.open();  // 打开串口
  }
  catch (serial::IOException &e)  // 抓取异常
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to open port ");  // 串口打开失败错误日志
    exit(0);  // 退出程序
  }
  if (serial_.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"Serial Port initialized");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to initial Serial port ");
    exit(0);
  }
  processLoop();
}

/**
 * @brief AHRS设备驱动节点析构函数
 * 关闭串口连接，清理资源
 */
ahrsBringup::~ahrsBringup()  // 析构函数关闭串口通道
{
  if (serial_.isOpen())
    serial_.close();  // 关闭串口连接
}

/**
 * @brief 主要数据处理循环
 * 从串口持续读取AHRS设备数据，进行解析校验，并发布相关话题
 */
void ahrsBringup::processLoop()  // 数据处理过程
{
  RCLCPP_INFO(this->get_logger(),"ahrsBringup::processLoop: start");  // 记录循环开始日志
  while (rclcpp::ok())  // 持续运行直到ROS2关闭
  {
    if (!serial_.isOpen())  // 检查串口是否打开
    {
      RCLCPP_WARN(this->get_logger(),"serial unopen");  // 串口未打开警告
    }
    //check head start   检查起始 数据帧头
    uint8_t check_head[1] = {0xff};  // 帧头检查缓冲区
    size_t head_s = serial_.read(check_head, 1);  // 从串口读取1字节数据
    if (if_debug_)
    {
      if (head_s != 1)
      {
        RCLCPP_ERROR(this->get_logger(),"Read serial port time out! can't read pack head.");
      }
      std::cout << std::endl;
      std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
    }
    if (check_head[0] != FRAME_HEAD)  // 验证帧头是否为正确的起始字节
    {
      continue;  // 帧头不正确，继续读取下一个字节
    }
    //check head type   检查数据类型
    uint8_t head_type[1] = {0xff};  // 数据类型检查缓冲区
    size_t type_s = serial_.read(head_type, 1);  // 读取数据类型字节
    if (if_debug_)
    {
      std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;  // 调试模式下打印数据类型
    }
    if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != TYPE_GEODETIC_POS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND&& head_type[0] != 0xff)
    {
      RCLCPP_WARN(this->get_logger(),"head_type error: %02X",head_type[0]);  // 数据类型错误警告
      continue;  // 数据类型不支持，继续读取
    }
    //check head length    检查对应数据类型的长度是否符合
    uint8_t check_len[1] = {0xff};  // 数据长度检查缓冲区
    size_t len_s = serial_.read(check_len, 1);  // 读取数据长度字节
    if (if_debug_)
    {
      std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;  // 调试模式下打印数据长度
    }
    // 验证数据长度与预期是否匹配
    if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)  // IMU数据类型长度检查
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (imu)");  // IMU数据长度错误警告
      continue;
    }else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)  // AHRS数据类型长度检查 0x30 48
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (ahrs)");  // AHRS数据长度错误警告
      continue;
    }else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)  // INSGPS数据类型长度检查
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (insgps)");  // INSGPS数据长度错误警告
      continue;
    }else if (head_type[0] == TYPE_GEODETIC_POS && check_len[0] != GEODETIC_POS_LEN)  // 地理位置数据类型长度检查
    {
      RCLCPP_WARN(this->get_logger(),"head_len error (GEODETIC_POS)");  // 地理位置数据长度错误警告
      continue;
    }
    else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50) // 未知数据，防止记录失败
    {
      uint8_t ground_sn[1];  // 未知数据类型序列号缓冲区
      size_t ground_sn_s = serial_.read(ground_sn, 1);  // 读取序列号
      if (++read_sn_ != ground_sn[0])  // 检查序列号是否连续
      {
        if ( ground_sn[0] < read_sn_)  // 处理序列号回绕情况
        {
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");  // 检测到序列号丢失警告
          }
          sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);  // 累加丢失的序列号
          read_sn_ = ground_sn[0];
          // continue;
        }
        else  // 正常序列号增加
        {
          if(if_debug_){
            RCLCPP_WARN(this->get_logger(),"detected sn lost.");  // 检测到序列号丢失警告
          }
          sn_lost_ += (int)(ground_sn[0] - read_sn_);  // 累加丢失的序列号
          read_sn_ = ground_sn[0];
          // continue;
        }
      }
      uint8_t ground_ignore[500];  // 未知数据丢弃缓冲区
      size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4));  // 丢弃未知数据
      continue;  // 继续处理下一帧
    }
    //read head sn 读取帧头序列号和校验码
    uint8_t check_sn[1] = {0xff};  // 序列号检查缓冲区
    size_t sn_s = serial_.read(check_sn, 1);  // 读取序列号
    uint8_t head_crc8[1] = {0xff};  // CRC8校验码缓冲区
    size_t crc8_s = serial_.read(head_crc8, 1);  // 读取CRC8校验码
    uint8_t head_crc16_H[1] = {0xff};  // CRC16高字节缓冲区
    uint8_t head_crc16_L[1] = {0xff};  // CRC16低字节缓冲区
    size_t crc16_H_s = serial_.read(head_crc16_H, 1);  // 读取CRC16高字节
    size_t crc16_L_s = serial_.read(head_crc16_L, 1);  // 读取CRC16低字节
    if (if_debug_)
    {
      std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
      std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
      std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
      std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
    }
    // put header & check crc8 & count sn lost
    // check crc8 进行crc8数据校验
    if (head_type[0] == TYPE_IMU)
    {
      imu_frame_.frame.header.header_start   = check_head[0];
      imu_frame_.frame.header.data_type      = head_type[0];
      imu_frame_.frame.header.data_size      = check_len[0];
      imu_frame_.frame.header.serial_num     = check_sn[0];
      imu_frame_.frame.header.header_crc8    = head_crc8[0];
      imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
      if (CRC8 != imu_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = imu_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      //check sn 
      ahrsBringup::checkSN(TYPE_IMU);
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      ahrs_frame_.frame.header.header_start   = check_head[0];
      ahrs_frame_.frame.header.data_type      = head_type[0];
      ahrs_frame_.frame.header.data_size      = check_len[0];
      ahrs_frame_.frame.header.serial_num     = check_sn[0];
      ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
      ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
      if (CRC8 != ahrs_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      //check sn 
      ahrsBringup::checkSN(TYPE_AHRS);
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      insgps_frame_.frame.header.header_start   = check_head[0];
      insgps_frame_.frame.header.data_type      = head_type[0];
      insgps_frame_.frame.header.data_size      = check_len[0];
      insgps_frame_.frame.header.serial_num     = check_sn[0];
      insgps_frame_.frame.header.header_crc8    = head_crc8[0];
      insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
      if (CRC8 != insgps_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      else if(if_debug_)
      {
        std::cout << "header_crc8 matched." << std::endl;
      }
      
      ahrsBringup::checkSN(TYPE_INSGPS);
    }
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      Geodetic_Position_frame_.frame.header.header_start   = check_head[0];
      Geodetic_Position_frame_.frame.header.data_type      = head_type[0];
      Geodetic_Position_frame_.frame.header.data_size      = check_len[0];
      Geodetic_Position_frame_.frame.header.serial_num     = check_sn[0];
      Geodetic_Position_frame_.frame.header.header_crc8    = head_crc8[0];
      Geodetic_Position_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      Geodetic_Position_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(Geodetic_Position_frame_.read_buf.frame_header, 4);
      if (CRC8 != Geodetic_Position_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(),"header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = Geodetic_Position_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      
      ahrsBringup::checkSN(TYPE_GEODETIC_POS);
    }
    // check crc16 进行crc16数据校验
    if (head_type[0] == TYPE_IMU)
    {
      uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); //48+1
      
      uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
      if (if_debug_)
      {          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(imu).");
        continue;
      }
      else if(imu_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      }
      
    }
    else if (head_type[0] == TYPE_AHRS)  //0x41
    {
      uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      //读取AHRS数据
      size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
     
      uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(ahrs).");
        continue;
      }
      else if(ahrs_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      uint16_t head_crc16_l = insgps_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = insgps_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); //48+1
    
      uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(ahrs).");
        continue;
      }
      else if(insgps_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      } 
    }
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      uint16_t head_crc16_l = Geodetic_Position_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = Geodetic_Position_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(Geodetic_Position_frame_.read_buf.read_msg, (GEODETIC_POS_LEN + 1)); //24+1
     
      uint16_t CRC16 = CRC16_Table(Geodetic_Position_frame_.frame.data.data_buff, GEODETIC_POS_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(),"check crc16 faild(gps).");
        continue;
      }
      else if(Geodetic_Position_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(),"check frame end.");
        continue;
      }
    }
    // publish magyaw topic
    //读取IMU数据进行解析，并发布相关话题

    if (head_type[0] == TYPE_IMU)
    {
      // publish imu topic
      sensor_msgs::msg::Imu imu_data;
      imu_data.header.stamp = rclcpp::Node::now();
      imu_data.header.frame_id = imu_frame_id_.c_str();
      Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                                ahrs_frame_.frame.data.data_pack.Qx,
                                ahrs_frame_.frame.data.data_pack.Qy,
                                ahrs_frame_.frame.data.data_pack.Qz);
      Eigen::Quaterniond q_r =                          
          Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_rr =                          
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_xiao_rr =
          Eigen::AngleAxisd( PI/2, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( PI, Eigen::Vector3d::UnitX());
      if (device_type_ == 0)         //未经变换的原始数据
      {
        imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
        imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
        imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
        imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
        imu_data.angular_velocity.x = imu_frame_.frame.data.data_pack.gyroscope_x;
        imu_data.angular_velocity.y = imu_frame_.frame.data.data_pack.gyroscope_y;
        imu_data.angular_velocity.z = imu_frame_.frame.data.data_pack.gyroscope_z;
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      else if (device_type_ == 1)    //imu单品rclcpp标准下的坐标变换
      {
        
        Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
        imu_data.orientation.w = q_out.w();
        imu_data.orientation.x = q_out.x();
        imu_data.orientation.y = q_out.y();
        imu_data.orientation.z = q_out.z();
        imu_data.angular_velocity.x =  imu_frame_.frame.data.data_pack.gyroscope_x;
        imu_data.angular_velocity.y = -imu_frame_.frame.data.data_pack.gyroscope_y;
        imu_data.angular_velocity.z = -imu_frame_.frame.data.data_pack.gyroscope_z;
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = -imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = -imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      imu_pub_->publish(imu_data);
}
    //读取AHRS数据进行解析，并发布相关话题
    else if (head_type[0] == TYPE_AHRS)
    {
      geometry_msgs::msg::Pose2D pose_2d;
      pose_2d.theta = ahrs_frame_.frame.data.data_pack.Heading;
      mag_pose_pub_->publish(pose_2d);
      //std::cout << "YAW: " << pose_2d.theta << std::endl;
      geometry_msgs::msg::Vector3 Euler_angles_2d,Magnetic;  
      Euler_angles_2d.x = ahrs_frame_.frame.data.data_pack.Roll;
      Euler_angles_2d.y = ahrs_frame_.frame.data.data_pack.Pitch;
      Euler_angles_2d.z = ahrs_frame_.frame.data.data_pack.Heading;
      Magnetic.x = imu_frame_.frame.data.data_pack.magnetometer_x;
      Magnetic.y = imu_frame_.frame.data.data_pack.magnetometer_y;
      Magnetic.z = imu_frame_.frame.data.data_pack.magnetometer_z;

      Euler_angles_pub_->publish(Euler_angles_2d);
      Magnetic_pub_->publish(Magnetic);

    }

    //读取gps_pos数据进行解析，并发布相关话题
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      sensor_msgs::msg::NavSatFix gps_data;
      gps_data.header.stamp = rclcpp::Node::now();
      gps_data.header.frame_id = "navsat_link";
      gps_data.latitude = Geodetic_Position_frame_.frame.data.data_pack.Latitude / DEG_TO_RAD;
      gps_data.longitude = Geodetic_Position_frame_.frame.data.data_pack.Longitude / DEG_TO_RAD;
      gps_data.altitude = Geodetic_Position_frame_.frame.data.data_pack.Height;

      gps_pub_->publish(gps_data);
    } 
    //读取INSGPS数据进行解析，并发布相关话题
    else if (head_type[0] == TYPE_INSGPS)
    {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = rclcpp::Node::now(); 
      // odom_msg.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
      odom_msg.pose.pose.position.x = insgps_frame_.frame.data.data_pack.Location_North; //Position //位置
      odom_msg.pose.pose.position.y = insgps_frame_.frame.data.data_pack.Location_East;
      odom_msg.pose.pose.position.z = insgps_frame_.frame.data.data_pack.Location_Down;

      // odom_msg.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
      odom_msg.twist.twist.linear.x =  insgps_frame_.frame.data.data_pack.Velocity_North; //Speed in the X direction //X方向速度
      odom_msg.twist.twist.linear.y =  insgps_frame_.frame.data.data_pack.Velocity_East; //Speed in the Y direction //Y方向速度
      odom_msg.twist.twist.linear.z =  insgps_frame_.frame.data.data_pack.Velocity_Down;
      NED_odom_pub_->publish(odom_msg);

      geometry_msgs::msg::Twist speed_msg;
      speed_msg.linear.x =  insgps_frame_.frame.data.data_pack.BodyVelocity_X;
      speed_msg.linear.y =  insgps_frame_.frame.data.data_pack.BodyVelocity_Y;
      speed_msg.linear.z =  insgps_frame_.frame.data.data_pack.BodyVelocity_Z;   
      twist_pub_->publish(speed_msg);
		  

    }   
  }
}

void ahrsBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if(magyaw < 0)
  {
    magyaw = magyaw + 2 * PI;
  }
  // return magyaw;
}

void ahrsBringup::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    if (++read_sn_ != imu_frame_.frame.header.serial_num)
    {
      if ( imu_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num;
    break;

  case TYPE_AHRS:
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num;
    break;

  case TYPE_INSGPS:
    if (++read_sn_ != insgps_frame_.frame.header.serial_num)
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num;
    break;

  case TYPE_GEODETIC_POS:
    if (++read_sn_ != Geodetic_Position_frame_.frame.header.serial_num)
    {
      if ( Geodetic_Position_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - Geodetic_Position_frame_.frame.header.serial_num);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(Geodetic_Position_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          RCLCPP_WARN(this->get_logger(),"detected sn lost.");
        }
      }
    }
    read_sn_ = Geodetic_Position_frame_.frame.header.serial_num;
    break;

  default:
    break;
  }
}

} //namespace FDILink

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  FDILink::ahrsBringup bp;

  return 0;
}
