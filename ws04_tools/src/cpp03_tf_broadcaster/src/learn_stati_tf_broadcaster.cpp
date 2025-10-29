#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace tf2_ros;
using namespace geometry_msgs::msg;
// using namespace tf2;
/*
需求：编写静态坐标变换程序，执行时传入两个坐标系的相对位姿关系以及父子级坐标系id，
     程序运行发布静态坐标变换
     ros2 run 包 可执行程序名 x y z roll pitch yaw frame child_frame
流程：
    0.判断传入的参数是否合法
    1.创建广播对象
    2.组织并发布数据
*/
// 自定义节点类继承Node
class TFStaticBroadcaster : public rclcpp::Node
{
public:
    TFStaticBroadcaster(char const **argv) : Node("static_broadcaster_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "静态坐标变换发布者启动");
        static_broadcaster_ = std::make_shared<StaticTransformBroadcaster>(this);
        pub_static_tf(argv);
    }

private:
    std::shared_ptr<StaticTransformBroadcaster> static_broadcaster_;

    void pub_static_tf(char const **argv)
    {
        // 1.组织消息
        TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = argv[7]; // 父级坐标系id
        transform.child_frame_id = argv[8];  // 子级坐标系id
        // 2.设置偏移量
        transform.transform.translation.x = atof(argv[1]);
        transform.transform.translation.y = atof(argv[2]);
        transform.transform.translation.z = atof(argv[3]);

        // 3.设置四元数
        tf2::Quaternion qtn;
        qtn.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
        transform.transform.rotation.x = qtn.x();
        transform.transform.rotation.y = qtn.y();
        transform.transform.rotation.z = qtn.z();
        transform.transform.rotation.w = qtn.w();

        // 4.广播
        static_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 实例化节点并进入事件循环
    auto node = std::make_shared<TFStaticBroadcaster>(argv);
    rclcpp::spin(node);

    // 资源释放
    rclcpp::shutdown();
    return 0;
}