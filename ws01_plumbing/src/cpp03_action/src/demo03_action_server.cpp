#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/fibonacci.hpp"

using base_interfaces_demo::action::Fibonacci;
using std::placeholders::_1;
using std::placeholders::_2;

class FibonacciActionServer : public rclcpp::Node
{
public:
    FibonacciActionServer() : Node("fibonacci_action_server")
    {
        // 创建动作服务端
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
            std::bind(&FibonacciActionServer::handle_cancel, this, _1),
            std::bind(&FibonacciActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Fibonacci动作服务端已创建");
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    // 处理目标请求
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "接收到目标请求，计算 %d 项斐波那契数列", goal->order);
        (void)uuid;
        
        // 检查目标是否有效
        if (goal->order <= 0) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消请求
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "接收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 处理接受的目标
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        // 在新线程中执行实际任务
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

    // 执行计算任务
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        // 获取目标
        auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "开始执行，计算 %d 项斐波那契数列", goal->order);

        // 创建反馈和结果对象
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        // 初始化斐波那契序列
        feedback->partial_sequence.clear();
        feedback->partial_sequence.push_back(0);
        feedback->partial_sequence.push_back(1);

        // 发布初始反馈
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "发布了初始反馈");

        // 循环计算斐波那契数列
        for (int i = 2; i < goal->order; i++) {
            // 检查是否收到取消请求
            if (goal_handle->is_canceling()) {
                result->sequence = feedback->partial_sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "任务已被取消");
                return;
            }

            // 检查ROS是否仍在运行
            if (!rclcpp::ok()) {
                result->sequence = feedback->partial_sequence;
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "任务被中断");
                return;
            }

            // 计算下一个斐波那契数
            int32_t next_number = feedback->partial_sequence[i-1] + feedback->partial_sequence[i-2];
            feedback->partial_sequence.push_back(next_number);

            // 发布反馈
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "已发布第 %d 项: %d", i+1, next_number);

            // 控制执行速度，便于观察
            loop_rate_.sleep();
        }

        // 任务完成，发布结果
        if (rclcpp::ok()) {
            result->sequence = feedback->partial_sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "任务成功完成");
        }
    }

    rclcpp::Rate loop_rate_ = rclcpp::Rate(1); // 1 Hz
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FibonacciActionServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}