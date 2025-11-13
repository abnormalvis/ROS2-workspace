#include <memory>
#include <string>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

class TfListenerNode : public rclcpp::Node
{
public:
  TfListenerNode()
  : Node("tf_listener")
  {
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    // 参数：父帧、子帧（可通过参数修改）
    this->declare_parameter<std::string>("parent_frame", "world");
    this->declare_parameter<std::string>("child_frame", "child_frame");
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();

    timer_ = this->create_wall_timer(500ms, std::bind(&TfListenerNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "tf_listener started, parent: '%s', child: '%s'", parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      // 使用最新的可用变换
      transform = buffer_->lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

      auto &t = transform.transform.translation;
      auto &r = transform.transform.rotation;

      RCLCPP_INFO(this->get_logger(), "Transform %s -> %s: translation=(%.3f, %.3f, %.3f) rotation=(%.3f, %.3f, %.3f, %.3f)",
        transform.header.frame_id.c_str(), transform.child_frame_id.c_str(),
        t.x, t.y, t.z, r.x, r.y, r.z, r.w);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not get transform: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string parent_frame_;
  std::string child_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
