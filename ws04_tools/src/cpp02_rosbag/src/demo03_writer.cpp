/*
需求：录制控制乌龟运动的速度指令
流程：
    1.创建录制对象
    2.设置磁盘文件
    3.写数据（创建速度订阅方，回调函数中执行写出操作）
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosbag2_cpp/writer.hpp"

using namespace std::placeholders;
// 自定义节点类继承Node
class MyBagRecorder : public rclcpp::Node
{
public:
    MyBagRecorder() : Node("my_bag_recorder_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "录制节点启动成功！");
        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        writer_->open("my_bag");

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel",
            rclcpp::QoS(10),
            std::bind(&MyBagRecorder::do_write_msg, this, _1));
    }

private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    void do_write_msg(const std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        RCLCPP_INFO(this->get_logger(), "正在录制数据...");
        /*
        void write(
    const MessageT & message,
    const std::string & topic_name,
    const rclcpp::Time & time)
  {
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();

    rclcpp::Serialization<MessageT> serialization;
    serialization.serialize_message(&message, serialized_msg.get());
    return write(serialized_msg, topic_name, rosidl_generator_traits::name<MessageT>(), time);
  }
  */

        //(geometry_msgs::msg::Twist_<std::allocator<void>>, const char [16], const char [24], rclcpp::Time)
        writer_->write(msg, "turtle1/cmd_vel", "geometry_msgs/msg/Twist", this->now());
    }
};

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 实例化节点并进入事件循环
    auto node = std::make_shared<MyBagRecorder>();
    rclcpp::spin(node);

    // 资源释放
    rclcpp::shutdown();
    return 0;
}