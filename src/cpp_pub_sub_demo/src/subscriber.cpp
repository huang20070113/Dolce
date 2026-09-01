#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("cpp_listener"), message_count_(0) {
        // 创建订阅者
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "counter",
            10,
            std::bind(&SubscriberNode::listener_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "C++ 订阅者节点已启动");
    }

private:
    void listener_callback(const std_msgs::msg::Int32 & msg) {
        message_count_++;
        RCLCPP_INFO(this->get_logger(), "收到消息 #%d: %d",
                    message_count_, msg.data);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    int message_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
