/*
需求：读取bag文件数据，并将数据输出至终端
流程：
    1.创建一个回放对象
    2.设置被读取的文件
    3.读消息
    4.关闭文件，释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace rosbag2_cpp;
using namespace geometry_msgs::msg;
// 自定义节点类继承Node
class MyBagLoader : public rclcpp::Node
{
public:
    MyBagLoader() : Node("my_bag_loader_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "正在播放bag文件...");
        bag_loader = std::make_unique<Reader>();

        bag_loader->open("my_bag");

        while (bag_loader->has_next())
        {
            auto message = bag_loader->read_next<Twist>();
            RCLCPP_INFO(this->get_logger(), "线速度:%.2f, 角速度:%.2f", message.linear.x, message.angular.z);
        }
        bag_loader->close();
    }

private:
    std::unique_ptr<Reader> bag_loader;
};

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 实例化节点并进入事件循环
    auto node = std::make_shared<MyBagLoader>();
    rclcpp::spin(node);

    // 资源释放
    rclcpp::shutdown();
    return 0;
}