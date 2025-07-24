#include "rclcpp/rclcpp.hpp"
#include "xycar_msgs/msg/xycar_motor.hpp"
#include "control.hpp"
#include "shared_mem.hpp"

class MotorNode : public rclcpp::Node {
public:
    MotorNode() : Node("motor_node") {
        publisher_ = this->create_publisher<xycar_msgs::msg::XycarMotor>("xycar_motor", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorNode::publish_motor_msg, this));
    }

private:
    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    SharedData* shm_ptr = (SharedData*)mmap(0, sizeof(SharedData), PROT_READ, MAP_SHARED, shm_fd, 0);

    void publish_motor_msg() {
        auto msg = xycar_msgs::msg::XycarMotor();
        msg.angle = (shm_ptr->steering)*100;
        msg.speed = (shm_ptr->throttle)*100;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: angle=%.2f, speed=%.2f", msg.angle, msg.speed);
    }

    rclcpp::Publisher<xycar_msgs::msg::XycarMotor>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}