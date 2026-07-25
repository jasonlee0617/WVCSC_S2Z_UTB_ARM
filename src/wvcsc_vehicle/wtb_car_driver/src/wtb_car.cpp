#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <unordered_map>
#include "unistd.h"
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <ctime>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>  // 提供 std::chrono 时间类型
#include <can_msgs/msg/frame.hpp>

#include <chrono>
#include <thread>
#include <atomic>
#include "wtb_car_driver/msg/car_msg.hpp" // ROS2 自定义消息类型
#include "std_msgs/msg/string.hpp"

VCI_BOARD_INFO pInfo;//用来获取设备信息。
using namespace std;

float back_wheel_speed = 0.0f;   //上报的后轮速度
float turn_angle = 0.0f;         //上报的转向角度
float battery_level = 0.0f;      //上报的电量百分比
float error_flag = 0.0f;         //上报的错误状态
float left_speed = 0.0f;         //上报的后左轮速度
float right_speed = 0.0f;        //上报的后右轮速度

//阿克慢前后轮距离
#define WHELLBASE 0.54


class WtbCarNode : public rclcpp::Node
{
public:
    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   WtbCarNode
     * Description：构造函数
     * Input： 无
     * Output：无
     * Return：无
    ********************************************************************/
    WtbCarNode() : Node("wtb_car_node")
    {
        // --------核心参数校准（新增/修改）-----------
        // 1. 轮距（前后轴距离）：实际测量后修改
        this->declare_parameter<double>("WHEELBASE", 1.00);
        this->get_parameter("WHEELBASE", WHEELBASE);

         
        this->declare_parameter<std::string>("child_frame_id", "base_footprint");
        this->get_parameter("child_frame_id", child_frame_id_);
        
        // 2. 线速度校准系数：解决速度测量不准（如实际1m/s，传感器显示1.1m/s）
        this->declare_parameter<double>("vel_scale", 1.0);
        this->get_parameter("vel_scale", vel_scale_);
        
        // 3. 转向角零偏：解决转向角传感器零点误差（静止时应为0，若有偏差需修正）
        this->declare_parameter<double>("steer_offset", 0.0);
        this->get_parameter("steer_offset", steer_offset_);
        
        // 4. 最小有效速度：过滤微小噪声（根据实际情况调整）
        this->declare_parameter<double>("min_speed", 0.0005);
        this->get_parameter("min_speed", min_speed_);

        this->declare_parameter<double>("max_speed",1.0);
        this->get_parameter("max_speed", max_speed_);

        this->declare_parameter<float>("stop_time_threshold",3.0);
        this->get_parameter("stop_time_threshold", stop_time_threshold);

        RCLCPP_INFO(this->get_logger(), "校准参数: 轮距=%.3f, 速度系数=%.3f, 转向零偏=%.3f child_frame_id_=%s", 
                 WHEELBASE, vel_scale_, steer_offset_,child_frame_id_.c_str());

       
        

        // 初始化里程计数据
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        // 初始化时间
        last_time_ = this->get_clock()->now();

        // 最新的传感器数据（成员变量缓存）
        latest_linear_velocity = 0.0;  // m/s
        latest_steering_angle = 0.0;   // 弧度

         // 声明参数
        declare_parameter("can_sub_topic", "can_tx_2");
        declare_parameter("can_pub_topic", "can_rx_2");

        declare_parameter("frame_id", "base_link");

        
        // 获取参数
        std::string can_sub_topic;
        get_parameter("can_sub_topic", can_sub_topic);
        std::string can_pub_topic;
        get_parameter("can_pub_topic", can_pub_topic);
        
        // 订阅CAN话题
        can_sub = this->create_subscription<can_msgs::msg::Frame>( can_sub_topic, 50, std::bind(&WtbCarNode::can_tx_callback, this, std::placeholders::_1));
        
        // 订阅运行状态话题
        run_static_sub_ = create_subscription<std_msgs::msg::String>("run_static", 10,std::bind(&WtbCarNode::run_static_callback, this, std::placeholders::_1));

        //发布CAN数据
        can_pub = this->create_publisher<can_msgs::msg::Frame>(can_pub_topic, 50);  

        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 50, std::bind(&WtbCarNode::CmdVelCallback, this, std::placeholders::_1));
        
        twist_cmd_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>( "/twist_cmd", 50, std::bind(&WtbCarNode::TwistCmdVelCallback, this, std::placeholders::_1));

        //发布里程计数据
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/car_odom", 50);
        car_pub = this->create_publisher<wtb_car_driver::msg::CarMsg>("/wtb_car_message", 10);
        
        
    }

private:

    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   print_can_message
     * Description：打印CAN消息
     * Input：
     *      const can_msgs::msg::Frame& msg：ROS2 CAN消息类型
     * Output：无
     * Return：无
    ********************************************************************/
    void print_can_message(const can_msgs::msg::Frame& msg)
    {
        std::ostringstream oss;
        oss << "CAN Message Information:" << std::endl
            << "  - Header Stamp: " << msg.header.stamp.nanosec << std::endl
            << "  - ID: 0x" << std::hex << msg.id << std::dec << std::endl
            << "  - Is Extended: " << (msg.is_extended? "true" : "false") << std::endl
            << "  - Is RTR: " << (msg.is_rtr? "true" : "false") << std::endl
            << "  - Is Error: " << (msg.is_error? "true" : "false") << std::endl
            << "  - DLC: " << msg.dlc << std::endl
            << "  - Data:";
        for (int i = 0; i < msg.dlc; ++i) {
            oss << std::endl << "    - Byte " << i << ": 0x" << std::hex << static_cast<int>(msg.data[i]) << std::dec;
        }
        
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   can_tx_callback
     * Description：接收ROS2 can_tx话题回调函数   
     * Input：
     *  const can_msgs::msg::Frame::SharedPtr msg：ROS2 CAN消息类型
     * Output：无
     * Return：无
    ********************************************************************/
    void can_tx_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        if(msg->id == 0x18C4D2EF) //can发送02代表接收的数据
        {
            string gear;
            double speed, angle;
            // 解析数据
            ParseCtrlFb(msg->data.data(), gear, speed, angle);
            if(gear == "R")
            {
                speed = -speed;
            }
    
            angle = angle * M_PI / 180.0;

            latest_linear_velocity = speed;  // 缓存最新速度
            latest_steering_angle =  angle;         // 缓存最新转向角
            // 缓存传感器时间戳
            latest_stamp_ = msg->header.stamp;
            CalculateAndPublished();

            current_speed = speed;
            current_angle = angle;

            car_msg_pub->speed = speed;
            //角度转弧度
            car_msg_pub->angle = angle ;

            car_msg_pub->header.stamp = this->get_clock()->now();  // 当前时间戳
            car_msg_pub->header.frame_id = "base_link";           // 坐标系ID
            car_pub->publish(*car_msg_pub); 
        }
        else if(msg->id == 0x18C4E2EF) //电量
        {
            car_msg_pub->battery = msg->data[0];
        }
    }

    /********************************************************** 
    * Author: zcj 
    * Alter: zcj 
    * Function:  BuildCtrlCmdData 
    * Description: 构建ctrl_cmd控制命令数据 
    * Input: 
    *       std::string& TargetGear : 目标档位 
    *       float TargetSpeed：目标速度 
    *       float TargetSteerAngle：目标转向角度 
    * Output:无 
    * Return: std::vector<unsigned char> : 构建的CAN控制命令数据 
    **********************************************************/
    std::vector<unsigned char> BuildCtrlCmdData(const std::string& TargetGear, float TargetSpeed, float TargetSteerAngle) 
    {
        // 初始化 8 字节数据
        std::vector<unsigned char> data(8, 0);

        // 目标档位：起始字节 0，起始位 0，信号长度 4，Unsigned
        std::unordered_map<std::string, unsigned char> GearMapping = {
            {"disable", 0x00},
            {"P", 0x01},
            {"R", 0x02},
            {"N", 0x03},
            {"D", 0x04}
        };
        unsigned char GearValue = GearMapping[TargetGear];
        data[0] = GearValue & 0xF;  // 低 4 位存档位

        // 目标车体速度：起始字节 0，起始位 4，信号长度 16，Unsigned，精度 0.001m/s/bit
        unsigned int SpeedInt = static_cast<unsigned int>(TargetSpeed * 1000);
        data[0] |= (SpeedInt & 0x0F) << 4;  // 速度高 4 位存到字节 0 高 4 位
        data[1] = (SpeedInt & 0x0FF0) >> 4;  // 速度中间 8 位存到字节 1
        data[2] = (SpeedInt & 0xF000) >> 12;  // 速度低 4 位存到字节 2

        // 目标车体转向角：起始字节 2，起始位 20，信号长度 16，signed，精度 0.01°/bit
        int SteerAngleInt = static_cast<int>(TargetSteerAngle / 0.01);
        data[2] |= (SteerAngleInt & 0x0F) << 4;  // 速度高 4 位存到字节 2 高 4 位
        data[3] = (SteerAngleInt & 0x0FF0) >> 4;  // 速度中间 8 位存到字节 3
        data[4] = (SteerAngleInt & 0xF000) >> 12;  // 速度低 4 位存到字节 4

        // Alive Rolling Counter：起始字节 6，起始位 52，信号长度 4，Unsigned
        data[6] = (AliveCounter << 4) & 0xF0;  // 低 4 位存心跳计数
        AliveCounter = (AliveCounter + 1) % 16;  // 循环计数，4 位最大值 15

        // 计算 Check BCC（消息异或校验）：Byte0 XOR Byte1 XOR Byte2 XOR Byte3 XOR Byte4 XOR Byte5 XOR Byte6
        unsigned char checksum = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];
        data[7] = checksum;  // 校验和存到第 7 字节

        return data;
    }

    /********************************************************** 
    * Author: zcj 
    * Alter: zcj 
    * Function:  ParseCtrlFb 
    * Description: 解析ctrl_fb报文数据 
    * Input: 
    *       const unsigned char data[8] : CAN报文数据 
    *       std::string &gear : 解析出的档位 
    *       double &speed : 解析出的速度 
    *       double &angle : 解析出的转向角度 
    * Output:无 
    * Return:无 
    **********************************************************/
    void ParseCtrlFb(const unsigned char data[8],std::string &gear,double &speed,double &angle) 
    {
        try {
            // 1. 目标档位：起始字节 0，起始位 0，长度 4 位（Unsigned）
            unsigned char GearRaw = data[0] & 0x0F;  // 掩码提取低 4 位
            // 枚举映射
            std::unordered_map<unsigned char, std::string> GearMap = {
                {0, "disable"},
                {1, "P"},
                {2, "R"},
                {3, "N"},
                {4, "D"}
            };
            gear = GearMap.count(GearRaw) ? GearMap[GearRaw] : "未知(" + std::to_string(GearRaw) + ")";

            // 2. 当前车体速度反馈：起始字节 0，起始位 4，长度 16 位（Unsigned，精度 0.001m/s/bit）
            unsigned int SpeedLow = (data[0] >> 4);  // 字节 0 高 4 位
            unsigned int SpeedMid = data[1];         // 字节 1
            unsigned int SpeedHigh = data[2] & 0x0F; // 字节 2 低 4 位
            unsigned int SpeedRaw = (SpeedHigh << 12) | (SpeedMid << 4) | SpeedLow;
            speed = SpeedRaw * 0.001;  // 0.001 m/s/bit

            // 3. 转向角（起始位 20，共 16 位，有符号，小端）
            int AngleLow = (data[2] >> 4);  // 字节 2 高 4 位
            int AngleMid = data[3];         // 字节 3
            int AngleHigh = data[4] & 0x0F; // 字节 4 低 4 位
            int AngleRaw = (AngleHigh << 12) | (AngleMid << 4) | AngleLow;

            // 处理负数（有符号数）
            if (AngleRaw & 0x8000) {  // 最高位为 1，表示负数
                AngleRaw = AngleRaw - 0x10000;  // 转换为负数
            }
            angle = AngleRaw * 0.01;  // 精度转换

                 
        } catch (const std::exception& e) {
            std::cerr << "ParseCtrlFb error: " << e.what() << std::endl;
            // return {"", 0.0, 0.0};
        }
    }

    /********************************************************** 
    * Author: zcj
    * Alter: zcj
    * Function:  SendSpeedToAKM
    * Description: 发送速度和角度控制信息指令到阿克曼底盘
    * Input: 
    *       float lineSpeed : 要控制的线速度
    *       float angle：转弯角度
    * Output:无 
    * Return:无 
    **********************************************************/
    void SendSpeedToAKM(float lineSpeed,float angle)
    {
        // RCLCPP_INFO(this->get_logger(), "SendSpeedToAKM: lineSpeed = %.3f, angle = %.3f", lineSpeed, angle);
        int Angle=0; 
        Angle = angle * 180 / M_PI;

        // 关键：用局部变量（非智能指针），每次循环重新构造消息，避免重复使用隐患
        can_msgs::msg::Frame msg;  // 改为栈上对象，无需shared_ptr
        msg.header.stamp = this->get_clock()->now();
        msg.id = 0x18C4D2D0;
        msg.is_extended = true;
        msg.is_rtr = false;
        msg.is_error = false;
        msg.dlc = 8;

        std::string TargetGear = "D";
        if(lineSpeed < 0)
        {
            TargetGear = "R";
            lineSpeed = -lineSpeed;
            Angle = -Angle;
        }

        // 连续发送3帧
        for(int i = 0; i < 3; i++)
        {
            // 构建当前帧数据（每次循环重新构建，心跳计数器会自动递增）
            std::vector<unsigned char> data = BuildCtrlCmdData(TargetGear, lineSpeed, Angle);
            // 用memcpy快速复制（比循环赋值高效）
            memcpy(msg.data.data(), data.data(), 8);
            // 发布消息（best_effort模式下立即返回）
            can_pub->publish(msg);  // 直接传递栈上对象，ROS 2会自动拷贝
        }
    }

    /********************************************************** 
    * Author: zcj 
    * Alter: zcj 
    * Function:  CmdVelCallback
    * Description: cmd_vel话题消息回调函数
    * Input: 
    *       float lineSpeed : 要控制的线速度
    *       float angle：转弯角度
    * Output:无 
    * Return:无 
    **********************************************************/
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)//速度控制回调
    {
        // m/s 和 rad/s
        // RCLCPP_INFO(this->get_logger(), "CmdVelCallback");

        if(enable_run_ == false)
        {
            rclcpp::Time nowT = rclcpp::Node::now();
            float index_time = (nowT - enable_last_time).seconds();  //Retrieves time interval, which is used to integrate 
            if(stop_time_threshold >= 0)
            {
                if(index_time>stop_time_threshold)
                {
                    enable_run_  = true;
                    RCLCPP_INFO(rclcpp::get_logger("string_subscriber"), "时间到，恢复运行");
                    RCLCPP_INFO(get_logger(), "时间到，恢复运行");
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }

        
        if(twist_aux->linear.x > max_speed_)
        {
            twist_aux->linear.x = max_speed_;
        }
        SendSpeedToAKM(twist_aux->linear.x, twist_aux->angular.z);
    }

    /********************************************************** 
    * Author: zcj 
    * Alter: zcj 
    * Function:  twist_CmdVelCallback 
    * Description: twist_cmd话题消息回调函数 
    * Input: 
    *       const geometry_msgs::msg::TwistStamped::SharedPtr &twist_aux : 速度控制消息 
    * Output:无 
    * Return:无 
    **********************************************************/
    void TwistCmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_aux)//速度控制回调
    {
        // rclcpp::Time send_time = twist_aux->header.stamp;
        // rclcpp::Time receive_time = this->get_clock()->now();

        // // 计算网络/传输延迟
        // double latency_sec = (receive_time - send_time).seconds();


        // RCLCPP_INFO(this->get_logger(), "发送时间: %f.%09ld, 接收时间: %f.%09ld, 延迟: %.6f s",
        //             send_time.seconds(),    // double -> %f
        //             send_time.nanoseconds(), // int64_t -> %ld
        //             receive_time.seconds(),  // double -> %f
        //             receive_time.nanoseconds(), // int64_t -> %ld
        //             latency_sec);



        if(twist_aux->twist.linear.x > max_speed_)
        {
            twist_aux->twist.linear.x = max_speed_;
        }
        //速度和角度
        // SendSpeedToAKM(twist_aux->twist.linear.x, angz_to_angle(twist_aux->twist.linear.x, twist_aux->twist.angular.z));
        double speed = twist_aux->twist.linear.x;
        double angle = twist_aux->twist.angular.z;
        // RCLCPP_INFO(this->get_logger(), "速度: %.3f, 角度: %.3f", speed, angle);
        SendSpeedToAKM(speed, angle);
    }

    /********************************************************** 
    * Author: zcj 
    * Alter: zcj 
    * Function:  angz_to_angle 
    * Description: 根据线速度和角速度计算转向角度 
    * Input: 
    *       float Vx : 线速度 
    *       float Vz : 角速度 
    * Output:无 
    * Return: float : 计算出的转向角度 
    **********************************************************/
    float angz_to_angle(float Vx,float Vz)
    {
        float R=0;
        if(Vx==0 || Vz==0)
        {
        return 0;
        }
        
        R = Vx/Vz;
        return 0.7*atan(WHELLBASE/R);
    }


    void publishOdom(double x, double y, double theta, double vx, double wz)
    {
        auto current_time = this->get_clock()->now();
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
 
        odom.header.frame_id = "odom";
        odom.child_frame_id = child_frame_id_.c_str();
        

        // 设置位置
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        
        // 设置机器人的偏航角（使用四元数表示）
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = sin(theta / 2.0);
        odom.pose.pose.orientation.w = cos(theta / 2.0);

        // 设置速度
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = wz;

        // Provide non-zero covariance so downstream fusion does not see a singular measurement.
        odom.pose.covariance.fill(0.0);
        odom.pose.covariance[0] = 0.05;   // x
        odom.pose.covariance[7] = 0.05;   // y
        odom.pose.covariance[14] = 9999.0;  // z unused in 2D
        odom.pose.covariance[21] = 9999.0;  // roll unused in 2D
        odom.pose.covariance[28] = 9999.0;  // pitch unused in 2D
        odom.pose.covariance[35] = 0.10;  // yaw

        odom.twist.covariance.fill(0.0);
        odom.twist.covariance[0] = 0.04;    // vx
        odom.twist.covariance[7] = 9999.0;  // vy unused
        odom.twist.covariance[14] = 9999.0; // vz unused
        odom.twist.covariance[21] = 9999.0; // wx unused
        odom.twist.covariance[28] = 9999.0; // wy unused
        odom.twist.covariance[35] = 0.04;   // wz

        // 发布里程计数据
        odom_pub_->publish(odom);
        
       
    }

    /**
     * @brief 角度归一化到[-π, π]
     */
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // 定时器回调：固定频率计算并发布里程计
    void timerCallback()
    {
        CalculateAndPublished();
    }

    void CalculateAndPublished()
    {
        rclcpp::Time current_time =  this->get_clock()->now();

        double dt = (current_time - last_time_).seconds();  // 实际时间差（更准确）

        if (dt <= 0 ) 
        {  
            // 假设传感器频率≥20Hz，dt应≤0.05s
            // last_time_ = current_time;
            RCLCPP_WARN(this->get_logger(), "dt: %lf is out of range, skip", dt);
            return;
        }

        // 优化：dt 超阈值时，仍计算一次里程计（避免丢位置），但限制最大 dt（防止积分发散）
        const double MAX_DT = 0.1;  // 最大允许 dt（0.1s，对应 10Hz 下限，可根据需求调整）
        if (dt > MAX_DT) {
            dt = MAX_DT;  // 超过最大 dt 时，按最大 dt 计算（避免积分误差过大）
        }

        // 1. 线速度校准：修正传感器测量误差
        double linear_velocity = latest_linear_velocity * vel_scale_;
        
        // 2. 转向角校准：移除零偏 + 限制最大角度
        double steering_angle = latest_steering_angle - steer_offset_;  // 移除零偏
        // RCLCPP_INFO(this->get_logger(), "steering_angle: %lf", steering_angle);
        // double max_delta = M_PI / 6;  // 30度（根据底盘机械极限调整）
        // double delta = std::max(std::min(steering_angle, max_delta), -max_delta);
         double delta = steering_angle;
        // 单位转换和里程计计算（和之前逻辑一致）
        double v = linear_velocity;  // m/s

        const double MIN_SPEED = 0.005;  // 小于0.005m/s视为静止
        if (fabs(v) < MIN_SPEED) 
        {
            v = 0.0;  // 静止时不产生位移
        }

        // 计算角速度（阿克曼模型：omega = v * tan(delta) / 轮距）
        //表示车辆朝向
        double omega = 0.0;
        if (fabs(WHEELBASE) > 1e-6) 
        {  // 避免除以零
            omega = v * tan(delta) / WHEELBASE;
        }

        // 更新位置和角度
        // 积分计算位移（使用当前theta_，避免延迟）
        double dx = v * cos(theta_) * dt;
        double dy = v * sin(theta_) * dt;
        x_ += dx;
        y_ += dy;

        // RCLCPP_INFO(this->get_logger(), "x_: %lf, y_: %lf, dx: %lf, dy: %lf,v: %lf, omega: %lf", x_, y_, dx, dy, v, omega);

        theta_ += omega * dt;
        // 角度归一化（关键：避免角度值无限累积）
        theta_ = normalizeAngle(theta_);

        last_time_ = current_time;

        // 发布里程计
        publishOdom(x_, y_, theta_, v, omega);
    }

    // 回调函数，处理接收到的使能消息
    void run_static_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == last_run_static_command_)
        {
            return;
        }

        last_run_static_command_ = msg->data;
        RCLCPP_INFO(rclcpp::get_logger("string_subscriber"), "Received: '%s'", msg->data.c_str());

        if (msg->data == "start")
        {
            RCLCPP_INFO(rclcpp::get_logger("string_subscriber"), "Received 'start' command 继续");
            if(enable_run_ == false)
            {
                enable_run_ = true;
            }
        }
        else if (msg->data == "stop")
        {
            RCLCPP_INFO(rclcpp::get_logger("string_subscriber"), "Received 'stop' command 停止");
            enable_last_time = rclcpp::Node::now();
            // 添加 'stop' 的逻辑
            if(enable_run_ == true)
            {
                enable_run_ = false;
                stop();
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("string_subscriber"), "Received unknown command");
        }
    }

    void stop()
    {
        SendSpeedToAKM(0.0, 0.0);
    }


    
    //定义can通道数据订阅节点
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
    // 订阅运行状态控制
   	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr run_static_sub_;

    //定义can数据发布节点
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_sub;

    std::shared_ptr<wtb_car_driver::msg::CarMsg> car_msg_pub = std::make_shared<wtb_car_driver::msg::CarMsg>();
    rclcpp::Publisher<wtb_car_driver::msg::CarMsg>::SharedPtr car_pub;

    
    //模拟心跳计数器
    unsigned char AliveCounter = 0;

     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


    double x_, y_, theta_;
    rclcpp::Time last_time_;
    rclcpp::Time latest_stamp_;

      // 缓存最新的传感器数据
    double latest_linear_velocity;  // m/s
    double latest_steering_angle;   // 最新转向角

    // --------------  新增校准参数 --------------
    double WHEELBASE;         // 轮距（前后轴距离，必须实际测量）
    double vel_scale_;        // 线速度校准系数（修正速度测量误差）
    double steer_offset_;     // 转向角零偏（修正转向传感器零点误差）
    double min_speed_;        // 最小有效速度（过滤噪声）
    double max_speed_;        // 最大速度

    std::string child_frame_id_;

    double current_speed = 0.0, current_angle = 0.0;

    bool enable_run_ = true; // 是否可以控制
    double stop_time_threshold = 3.0; // 停止时间阈值（秒）
    rclcpp::Time enable_last_time; //记录接收到stop信息的时间
    std::string last_run_static_command_;


  
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WtbCarNode>();
    // rclcpp::spin(node); //单线程执行器 同一时间只能运行一个回调函数。

    // 使用 MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    // 添加节点
    executor.add_node(node);
    // 设置线程数（建议设为 2 或 4，让处理 CAN 和处理 cmd_vel 的线程分开）
    // 注意：这个 API 在某些版本可能不需要，默认会根据 CPU 核心数自动分配
    // executor.spin_some(); 
    
    // 启动多线程旋转
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
