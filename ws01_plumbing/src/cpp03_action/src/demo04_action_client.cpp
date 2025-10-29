#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/fibonacci.hpp"

using base_interfaces_demo::action::Fibonacci;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
class FibonacciActionClient : public rclcpp::Node
{
public:
    FibonacciActionClient() : Node("fibonacci_action_client")
    {
        // 创建动作客户端
        action_client_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci");

        RCLCPP_INFO(this->get_logger(), "Fibonacci动作客户端已创建");
    }

    // 发送目标
    void send_goal(int32_t order)
    {
        // 等待服务端上线
        while (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待动作服务端时中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待动作服务端上线...");
        }

        // 创建目标消息
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = order;

        // 发送目标的选项设置
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&FibonacciActionClient::result_callback, this, _1);
        // 发送目标
        RCLCPP_INFO(this->get_logger(), "发送目标: 计算 %d 项斐波那契数列", order);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // 目标响应回调
    void goal_response_callback(
        const GoalHandleFibonacci::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "目标被服务端拒绝");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标被服务端接受");
        }
    }

    // 反馈回调
    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "收到反馈: ";
        for (const auto &number : feedback->partial_sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // 结果回调
    void result_callback(const GoalHandleFibonacci::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "目标被中止");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "目标被取消");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "未知结果码");
            return;
        }

        std::stringstream ss;
        ss << "最终结果: ";
        for (const auto &number : result.result->sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请输入参数：目标数");
        return 1;
    }
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FibonacciActionClient>();

    // 发送目标：计算10项斐波那契数列
    node->send_goal(10);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}