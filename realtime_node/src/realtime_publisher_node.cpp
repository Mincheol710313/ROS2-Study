#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <pthread.h>
#include <thread>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        auto timer_callback = 
        [this]() -> void {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(this->count_++);
            this->publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(1ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int count_;
};

int main(int argc, char * argv[])
{   
    struct sched_param param;
    param.sched_priority = 99;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();
    auto spin_thread = std::thread(
        [&](){
            rclcpp::spin(node);
        });
    pthread_setschedparam(spin_thread.native_handle(), SCHED_FIFO,&param);
    
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}