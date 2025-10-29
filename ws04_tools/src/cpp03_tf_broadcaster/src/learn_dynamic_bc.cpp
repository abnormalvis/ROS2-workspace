/*
需求：启动 turtlesim_node 节点，编写程序，发布乌龟(turtle1)相对于窗体(world)的位姿
流程：
    1.创建动态广播器
    2.创建乌龟位姿订阅方
    3.回调函数中获取乌龟位姿，成相对关系，并发布
*/

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <turtlesim/msg/pose.hpp>

using std::placeholders::_1;
using turtlesim::msg::Pose;
using tf2::Quaternion;
// 自定义节点类继承Node
class TFDynamicBroadcaster : public rclcpp::Node
{
public:
    TFDynamicBroadcaster() : Node("hello_node_cpp")
    {
        // 1.创建指针对象
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // 2.创建乌龟位姿订阅方
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TFDynamicBroadcaster::pose_sub_callback, this, _1));

        
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<rclcpp::Subscription<turtlesim::msg::Pose>> pose_sub_;

    void pose_sub_callback(const Pose &pose)
    {
        RCLCPP_INFO(this->get_logger(), "准备发布turtle1的动态tf变换");
        // 1.组织消息
        geometry_msgs::msg::TransformStamped dynamic_transform;
        dynamic_transform.header.stamp = this->now();
        dynamic_transform.header.frame_id = "world";
        dynamic_transform.child_frame_id = "turtle1";
        dynamic_transform.transform.translation.x = pose.x;
        dynamic_transform.transform.translation.y = pose.y;
        dynamic_transform.transform.translation.z = 0.0;
        
        //2.创建四元数
        Quaternion qtn;
        qtn.setRPY(0,0,pose.theta);
        dynamic_transform.transform.rotation.x = qtn.x();
        dynamic_transform.transform.rotation.y = qtn.y();
        dynamic_transform.transform.rotation.z = qtn.z();
        dynamic_transform.transform.rotation.w = qtn.w();

        tf_broadcaster_->sendTransform(dynamic_transform);
    }
};

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 实例化节点并进入事件循环
    auto node = std::make_shared<TFDynamicBroadcaster>();
    
    rclcpp::spin(node);

    // 资源释放
    rclcpp::shutdown();
    return 0;
}