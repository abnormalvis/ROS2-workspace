#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
class TFBroadcasterPoint : public rclcpp::Node
{
  public:
    TFBroadcasterPoint() : Node("tf_broadcaster_point")
    { 
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      timer_ = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), 
        this->get_node_timers_interface());
      tf_buffer_->setCreateTimerInterface(timer_);
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      point_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PointStamped>>(this, "point");
      
      /**
   * \brief Constructor
   *
   * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
   * \param buffer The buffer this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node_logging The logging interface to use for any log messages
   * \param node_clock The clock interface to use to get the node clock
   * \param buffer_timeout The timeout duration after requesting transforms from the buffer.
   */
      point_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
        *point_sub_, 
        *tf_buffer_, 
        "base_link", 
        10, 
        this->get_node_logging_interface(), 
        this->get_node_clock_interface(), 
        1s
      );
      // 解析数据
      point_filter_->registerCallback(
        std::bind(&TFBroadcasterPoint::pointCallback, this, std::placeholders::_1)
      );
    }
    
  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PointStamped>> point_sub_;
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> point_filter_;

    void pointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr & point_msg)
    {
      geometry_msgs::msg::TransformStamped transform;
      try
      {
        auto output = tf_buffer_->transform(*point_msg, "base_link");
        RCLCPP_INFO(this->get_logger(), "Received point: (%.2f, %.2f, %.2f) in frame %s",
          output.point.x, 
          output.point.y, 
          output.point.z, 
          output.header.frame_id.c_str()
        );
      }
      catch (tf2::TransformException & ex)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      }
    }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcasterPoint>());
  return 0;
}