#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// 使用命名空间
using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("cpp_talker"), counter_(0) {
        // 创建发布者
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);

        // 创建定时器，每秒执行一次
        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&PublisherNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "C++ 发布者节点已启动");
    }

private:
    void timer_callback() {
        auto msg = std_msgs::msg::Int32();
        msg.data = counter_++;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "发送消息: %d", msg.data);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char *argv[]) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建并运行节点
    rclcpp::spin(std::make_shared<PublisherNode>());

    // 关闭ROS2
    rclcpp::shutdown();

    return 0;
}

