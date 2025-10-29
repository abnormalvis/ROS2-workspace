/*
需求：创建参数客户端，查询或修改服务端参数
自定义节点类：
    1.创建参数客户端对象
    2.连接服务端
    3.参数查询
    4.修改参数
*/
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
// 自定义节点类继承Node
class MyParamClient : public rclcpp::Node
{
public:
    MyParamClient() : Node("myparam_client_node_cpp")
    {
        // static inline rclcpp::SyncParametersClient::SharedPtr rclcpp::SyncParametersClient::make_shared<MyParamClient *, const char (&)[21]>(MyParamClient *&&args, const char (&args)[21])
        sync_param_client_ = rclcpp::SyncParametersClient::make_shared(this, "my_param_server_node");
    }

    bool connect_server()
    {
        while (!sync_param_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }
        return true;
    }

    void get_param()
    {
        auto param_list = sync_param_client_->get_parameters({"stu_name", "stu_age"});
        // 获取某个参数
        std::string stu_name = sync_param_client_->get_parameter<std::string>("car_name");
        double stu_age = sync_param_client_->get_parameter<double>("stu_age");
        RCLCPP_INFO(this->get_logger(), "stu_name = %s", stu_name.c_str());
        RCLCPP_INFO(this->get_logger(), "stu_age = %.2f", stu_age);
    }

    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----参数修改操作------");
        sync_param_client_->set_parameters<>({rclcpp::Parameter("stu_name", "dragonB"),
                                              rclcpp::Parameter("stu_age", 666),
                                              rclcpp::Parameter("length", 114514)});

        // 可以设置先前不存在的参数，但前提为服务端设置了rclcpp::NodeOptions().allow_undeclared_parameters(true)
        RCLCPP_INFO(this->get_logger(), "新设置的参数：%.2f", sync_param_client_->get_parameter<double>("length"));
    }

private:
    rclcpp::SyncParametersClient::SharedPtr sync_param_client_;
};

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 实例化节点并进入事件循环
    auto node = std::make_shared<MyParamClient>();
    rclcpp::spin(node);
    bool flag = node->connect_server();
    if (!flag)
    {
        return 0;
    }
    node->get_param();
    node->update_param();
    node->get_param();
    // 资源释放
    rclcpp::shutdown();
    return 0;
}