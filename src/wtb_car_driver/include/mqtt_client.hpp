#include <iostream>
#include <thread>    // 包含 std::this_thread
#include <chrono>    // 包含 std::chrono
#include <mosquittopp.h>
#include <string>
#include <functional>
class MyMqttClient : public mosqpp::mosquittopp {
public:
    // 定义回调函数类型：参数为主题和消息内容
    using MessageCallback = std::function<void(const std::string& topic, const std::string& payload)>;

    MyMqttClient(const char* id) ;

    void on_connect(int rc) override ;

    void setSubTopic(std::string topic);
   
    /*
    * mqtt 接收回调函数
    */
    void on_message(const struct mosquitto_message* message) override ;

    /*
     * mqtt 数据发布函数
     */
    void on_publish(int mid) override ;

     /*
     * mqtt 订阅回调函数
     */
    void on_subscribe(int mid, int qos_count, const int* granted_qos) override ;

    /*
     * mqtt 断开连接回调函数
     */
    void on_disconnect(int rc) override ;

    

    // 设置外部回调函数
    void setMessageCallback(MessageCallback callback) {
        msg_callback_ = std::move(callback);
    }

private:
    std::string topic_="test";
    MessageCallback msg_callback_; // 存储外部回调函数
};

