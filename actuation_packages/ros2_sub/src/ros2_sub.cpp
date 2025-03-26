#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <stdio.h>

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

class PoseSubscriber : public rclcpp::Node {
public:
    PoseSubscriber() : Node("dds_test_sub") {
        subscription_ = this->create_subscription<PoseStampedMsg>(
            "test_pose",
            10,
            [this](const PoseStampedMsg::SharedPtr msg) {
                handle_pose(*msg);
            });
    }

private:
    void handle_pose(const PoseStampedMsg& msg) {
        RCLCPP_INFO(get_logger(), "--------------------------------\n");
        RCLCPP_INFO(get_logger(), "Timestamp: %.6f\n", rclcpp::Time(msg.header.stamp).seconds());
        RCLCPP_INFO(get_logger(), "Frame ID: %s\n", msg.header.frame_id.c_str());
        RCLCPP_INFO(get_logger(), "Received pose: (%.1f, %.1f, %.1f)\n", 
                  msg.pose.position.x,
                  msg.pose.position.y,
                  msg.pose.position.z);
        RCLCPP_INFO(get_logger(), "--------------------------------\n");
    }

    rclcpp::Subscription<PoseStampedMsg>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSubscriber>();
    RCLCPP_INFO(node->get_logger(), "Node created\n");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 