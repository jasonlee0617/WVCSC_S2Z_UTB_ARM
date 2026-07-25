#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include "controlcan.h"
#include <chrono>
#include <thread>
#include <atomic>
/*******************************************************************
* Author:zcj
* Alter: zcj
* Class：   CanBridgeNode
* Description：CAN通信节点，接收CAN数据，转换为ROS2 话题发出，接收ROS2 话题，转换为CAN消息发送
********************************************************************/
class CanBridgeNode : public rclcpp::Node
{
public:
    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   CanBridgeNode
     * Description：构造函数
     * Input： 无
     * Output：无
     * Return：无
    ********************************************************************/
    CanBridgeNode() : Node("can_bridge_node")
    {
        // 订阅CAN话题
        can_sub_1 = this->create_subscription<can_msgs::msg::Frame>(
            "can_rx_1", 10, std::bind(&CanBridgeNode::can_rx_callback_1, this, std::placeholders::_1));
        
        can_sub_2 = this->create_subscription<can_msgs::msg::Frame>(
            "can_rx_2", 10, std::bind(&CanBridgeNode::can_rx_callback_2, this, std::placeholders::_1));
        

        // 发布CAN话题
        can_pub_1 = this->create_publisher<can_msgs::msg::Frame>("can_tx_1", 10);
        can_pub_2 = this->create_publisher<can_msgs::msg::Frame>("can_tx_2", 10);

        // 初始化CAN接口
        Init_Can();
        // 启动CAN 接收线程
        can_receive_thread_ = std::thread(&CanBridgeNode::can_receive_loop, this);


    }

    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   ~CanBridgeNode
     * Description：析构函数
     * Input： 无
     * Output：无
     * Return：无
    ********************************************************************/
    ~CanBridgeNode()
    {
        if (can_receive_thread_.joinable()) 
        {
            can_receive_thread_.join();
        }
    }

    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   Init_Can
     * Description：Can 初始化，开启can盒
     * Input：无
     * Output：无
     * Return：无
    ********************************************************************/
    void Init_Can()
    {
        //类型，索引，保留参数
        dwRel =VCI_OpenDevice(DevType,DevIndex,1);
        if(dwRel != 1)
        {
            can_f = 0;
            RCLCPP_INFO(this->get_logger(), "打开设备失败\n");
            return;
        }
        //清空指定CAN通道的缓冲区
        VCI_ClearBuffer(DevType,DevIndex,0);
        VCI_ClearBuffer(DevType,DevIndex,1);
        VCI_INIT_CONFIG vic;
        vic.AccCode=0x80000008;
        vic.AccMask=0xFFFFFFFF;
        vic.Filter=1;
        vic.Timing0=0x00;   //波特率500
        vic.Timing1=0x1c;
        vic.Mode=0;
        //类型，索引，通道，设置结构体
        dwRel =VCI_InitCAN(DevType,DevIndex, CANIndex, &vic);
        if(dwRel !=1)
        {
            RCLCPP_INFO(this->get_logger(), "通道1 初始化失败\n");
            can_f = 0;
            return;
        }
        //启动CAN卡的某一个CAN通道
        dwRel =VCI_StartCAN(DevType,DevIndex, CANIndex);
        if(dwRel != 1)
        {
            RCLCPP_INFO(this->get_logger(), "通道1 打开\n");
            can_f = 0;
            return;
        }

        dwRel =VCI_InitCAN(DevType,DevIndex, CANIndex2, &vic);
        if(dwRel !=1)
        {
            RCLCPP_INFO(this->get_logger(), "通道2 初始化失败\n");
            can_f = 0;
            return;
        }
        //启动CAN卡的某一个CAN通道
        dwRel =VCI_StartCAN(DevType,DevIndex, CANIndex2);
        if(dwRel != 1)
        {
            RCLCPP_INFO(this->get_logger(), "通道2 打开失败\n");
            can_f = 0;
            return;
        }
        can_f = 1;
    }

    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   Can_close
     * Description：关闭Can
     * Input：无
     * Output：无
     * Return：无
    ********************************************************************/
    void Can_close()
    {
        if(can_f == 1)
        {
            dwRel = VCI_CloseDevice(DevType,DevIndex);
        }
    }
    
    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   sendMsg1
     * Description：Can通道1发送数据，标准帧
     * Input：
     *           uint id：发送Can数据的帧ID
     *           unsigned char data[]：要发送的CAN数据
     * Output：无
     * Return：无
    ********************************************************************/
    void sendMsg1(uint id,unsigned char data[])
    {
        VCI_CAN_OBJ vco;
        vco.ID = id;
        vco.SendType = 1;
        vco.RemoteFlag = 0;
        vco.ExternFlag = 0;
        vco.DataLen = 8;
        memcpy(vco.Data,data,sizeof (unsigned char)*8);
        dwRel = VCI_Transmit(DevType,DevIndex, CANIndex, &vco,1);

    }
    
    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   sendMsg2
     * Description：Can通道2发送数据，标准帧
     * Input：
     *           uint id：发送Can数据的帧ID
     *           unsigned char data[]：要发送的CAN数据
     * Output：无
     * Return：无
    ********************************************************************/
    void sendMsg2(uint id,unsigned char data[])
    {
        VCI_CAN_OBJ vco;
        vco.ID = id;
        vco.SendType = 1;
        vco.RemoteFlag = 0;
        vco.ExternFlag = 0;
        vco.DataLen = 8;
        memcpy(vco.Data,data,sizeof (unsigned char)*8);
        dwRel = VCI_Transmit(DevType,DevIndex, CANIndex2, &vco,1);
    }
    

private:
    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   can_rx_callback_1
     * Description：接收ROS2 can_rx_1话题回调函数，转换为CAN消息发送到CAN总线上
     * Input：
     *  const can_msgs::msg::Frame::SharedPtr msg：ROS2 CAN消息类型
     * Output：无
     * Return：无
    ********************************************************************/
    void can_rx_callback_1(const can_msgs::msg::Frame::SharedPtr msg)
    {
        VCI_CAN_OBJ vco;
        vco.ID = msg->id;
        vco.SendType = 1;
        vco.RemoteFlag = msg->is_rtr;
        vco.ExternFlag = msg->is_extended;
        vco.DataLen = msg->dlc;
        memcpy(vco.Data,msg->data.begin(),sizeof (unsigned char)*8);
        // std::copy(msg->data.begin(), msg->data.end(), can_frame.data);
        dwRel = VCI_Transmit(DevType,DevIndex, CANIndex, &vco,1);
    }

    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   can_rx_callback_2
     * Description：接收ROS2 can_rx_2话题回调函数，转换为CAN消息发送到CAN总线上
     * Input：
     *  const can_msgs::msg::Frame::SharedPtr msg：ROS2 CAN消息类型
     * Output：无
     * Return：无
    ********************************************************************/
    void can_rx_callback_2(const can_msgs::msg::Frame::SharedPtr msg)
    {
        VCI_CAN_OBJ vco;
        vco.ID = msg->id;
        vco.SendType = 1;
        vco.RemoteFlag = msg->is_rtr;
        vco.ExternFlag = msg->is_extended;
        vco.DataLen = msg->dlc;
        memcpy(vco.Data,msg->data.begin(),sizeof (unsigned char)*8);
        dwRel = VCI_Transmit(DevType,DevIndex, CANIndex2, &vco,1);

         // 更新“上一帧时间”为当前帧时间（为下一次计算做准备）
        // last_send_time_ = current_send_time;
    }

    /*******************************************************************
     * Author:zcj
     * Alter: zcj
     * Function：   can_receive_loop
     * Description：线程函数，接收CAN总线消息，转换为ROS话题发送
     * Input：
     *  const can_msgs::msg::Frame::SharedPtr msg：ROS2 CAN消息类型
     * Output：无
     * Return：无
    ********************************************************************/
    void can_receive_loop()
    {
        // 将接收到的CAN消息转换为话题消息并发布
        while (rclcpp::ok()) 
        {
            // 添加 50 毫秒的延时
            // std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            can_receive_parse();

        }
    }

    void can_receive_parse()
    {
        int reclen=0;
        int recvNum = 2500; //接收缓存，设为3000为佳。
        VCI_CAN_OBJ rec[recvNum];

        //重新打开CAN设备
        if(can_f == 0 )
        {
            Init_Can();
            return;
        }

        //获取CAN1数据
        reclen=VCI_Receive(DevType,DevIndex,CANIndex,rec,recvNum,0);
        
        if(reclen == -1) //设备掉线
        {
            RCLCPP_INFO(this->get_logger(), "设备掉线\n");
            can_f = 0;
            return;
        }

        
        if( reclen >0 )//调用接收函数，如果有数据，进行数据处理显示。
        {
            for(int j=0;j<reclen;j++)
            {
                auto msg = std::make_shared<can_msgs::msg::Frame>();
                msg->header.stamp = this->get_clock()->now();
                msg->id = rec[j].ID;
                msg->is_extended = rec[j].ExternFlag;     //标准帧或扩展帧                                                                
                msg->is_rtr = rec[j].RemoteFlag;          //数据帧或远程帧
                msg->is_error = false;                    // 正常帧
                msg->dlc = rec[j].DataLen;                //数据长度
                //can数据
                std::copy(rec[j].Data, rec[j].Data + rec[j].DataLen, msg->data.begin());
                can_pub_1->publish(*msg);
            }
        }
        
        //获取CAN2数据
        reclen = VCI_Receive(DevType, DevIndex, CANIndex2, rec, recvNum, 0);
        
        // if( reclen >0 )//调用接收函数，如果有数据，进行数据处理显示。
        // {
        //     for(int j=0;j<reclen;j++)
        //     {
        //         auto msg = std::make_shared<can_msgs::msg::Frame>();
        //         msg->header.stamp = this->get_clock()->now();
        //         msg->id = rec[j].ID;
        //         msg->is_extended = rec[j].ExternFlag;     //标准帧或扩展帧                                                                
        //         msg->is_rtr = rec[j].RemoteFlag;          //数据帧或远程帧
        //         msg->is_error = false;                    // 正常帧
        //         msg->dlc = rec[j].DataLen;
        //         std::copy(rec[j].Data, rec[j].Data + rec[j].DataLen, msg->data.begin());
        //         can_pub_2->publish(*msg);
        //     }
        // }


       if (reclen > 0) {
        std::vector<can_msgs::msg::Frame> pub_msgs;
        pub_msgs.reserve(reclen);

        // 批量缓存消息
        for (int j = 0; j < reclen; j++) {
            can_msgs::msg::Frame msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "can2";
            msg.id = rec[j].ID;
            msg.is_extended = rec[j].ExternFlag;
            msg.is_rtr = rec[j].RemoteFlag;
            msg.is_error = false;
            msg.dlc = rec[j].DataLen;
            std::copy(rec[j].Data, rec[j].Data + rec[j].DataLen, msg.data.begin());
            pub_msgs.push_back(msg);
        }

        // Humble兼容的批量发布（循环publish）
        for (const auto& msg : pub_msgs) {
            can_pub_2->publish(msg);
        }
    }
        
        
    }

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
        RCLCPP_INFO(this->get_logger(), "CAN Message Information:");
        RCLCPP_INFO(this->get_logger(), "  - Header Stamp: %lu", msg.header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "  - ID: 0x%X", msg.id);
        RCLCPP_INFO(this->get_logger(), "  - Is Extended: %s", msg.is_extended ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - Is RTR: %s", msg.is_rtr ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - Is Error: %s", msg.is_error ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - DLC: %d", msg.dlc);
        RCLCPP_INFO(this->get_logger(), "  - Data:");
        for (int i = 0; i < msg.dlc; ++i) {
            RCLCPP_INFO(this->get_logger(), "    - Byte %d: 0x%X", i, msg.data[i]);
        }
    }

    //定时器
    rclcpp::TimerBase::SharedPtr timer_;

    //定义can通道1数据订阅节点
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_1;
    //定义can通道1数据发布节点
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_1;
    //定义can通道2数据订阅节点
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_2;
    //定义can通道2数据发布节点
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_2;
    //定义线程
    std::thread can_receive_thread_;
    //定义CAN配置
    DWORD DevType = 4;    //设备类型
    DWORD DevIndex = 0;    //索引号
    DWORD CANIndex = 0;      //通道 0 = can1 1 = can2
    DWORD CANIndex2 = 1;
    DWORD dwRel;
    //设备开启状态 0：关闭 1：打开
    bool can_f = 0;  

    // 新增：存储上一帧的发送时间（初始化为0，标记为“无历史帧”）
    std::chrono::high_resolution_clock::time_point last_send_time_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
