/*
需求：创建参数服务端，并操作参数（增删改查）
流程：
    自定义节点类
    1.增
    2.查
    3.改
    4.删
*/

#include "rclcpp/rclcpp.hpp"

// 自定义节点类继承Node
class MyParamServer : public rclcpp::Node
{
public:
    MyParamServer() : Node("myparam_server_node_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
        RCLCPP_INFO(this->get_logger(), "参数服务端已启动");
    }

    void declare_param()
    {
        RCLCPP_INFO(this->get_logger(), "声明参数");
        this->declare_parameter("stu_name", "tom");
        this->declare_parameter("stu_age", 18);
        this->declare_parameter("stu_height", 1.78);

        RCLCPP_INFO(this->get_logger(), "参数已声明");
    }

    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "获取参数");
        //获取指定参数
        auto stu_name = this->get_parameter("stu_name").as_string();
        RCLCPP_INFO(this->get_logger(), "stu_name = %s", stu_name.c_str());

        //获取一些参数
        auto params = this->get_parameters({"stu_name", "stu_age", "stu_height"});
        for(auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "参数：%s = %s", param.get_name().c_str(), param.value_to_string().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "参数已获取");
    }

    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "更新参数");
        if(this->has_parameter("stu_name") == true)
        {
            this->set_parameter(rclcpp::Parameter("stu_name", "jerry"));
        } else {
            RCLCPP_INFO(this->get_logger(), "未找到参数stu_name");
        }
        
    }

    void delete_param()
    {
        RCLCPP_INFO(this->get_logger(), "删除参数");
        RCLCPP_INFO(this->get_logger(), "是否包含参数数stu_name：%d", this->has_parameter("stu_name"));
        if(this->has_parameter("stu_name") == true)
        {
            this->undeclare_parameter("stu_name");
            RCLCPP_INFO(this->get_logger(), "参数stu_name已删除");
        } else {
            RCLCPP_INFO(this->get_logger(), "未找到参数stu_name");
        }
    }
private:

};

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 实例化节点并进入事件循环
    auto node = std::make_shared<MyParamServer>();
    node->declare_param();
    node->get_param();
    node->update_param();
    node->get_param();
    rclcpp::spin(node);

    // 资源释放
    rclcpp::shutdown();
    return 0;
}