#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class DvlRelayNode : public rclcpp::Node {
public:
    DvlRelayNode() : Node("dvl_relay_node") {
        dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/dvl/twist", 10, std::bind(&DvlRelayNode::dvlCallback, this, std::placeholders::_1));
        dvl_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/dvl/data", 10);
    }

private:
    void dvlCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto new_msg = std::make_shared<geometry_msgs::msg::TwistStamped>(*msg);
        new_msg->header.frame_id = "dvl_link";  // Set the appropriate frame ID
        dvl_pub_->publish(*new_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DvlRelayNode>());
    rclcpp::shutdown();
    return 0;
}
