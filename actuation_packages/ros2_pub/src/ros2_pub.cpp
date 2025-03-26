#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PosePublisher : public rclcpp::Node {
public:
    PosePublisher() : Node("dds_test_pub"), count_(0) {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("test_pose", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&PosePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = geometry_msgs::msg::PoseStamped();
        
        // Set header
        message.header.stamp = this->now();
        message.header.frame_id = "map";
        
        // Set position
        message.pose.position.x = static_cast<double>(count_);
        message.pose.position.y = static_cast<double>(count_ * 2.0);
        message.pose.position.z = static_cast<double>(count_ * 0.5);
        
        // Set orientation (unit quaternion)
        message.pose.orientation.w = 1.0;
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Publishing pose: (%f, %f, %f)",
                   message.pose.position.x,
                   message.pose.position.y,
                   message.pose.position.z);
                   
        publisher_->publish(message);
        count_++;
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}