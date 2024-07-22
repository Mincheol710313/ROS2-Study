#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <pthread.h>
#include <thread>

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
       auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscriber_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{   
    struct sched_param param;
    param.sched_priority = 99;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();
    auto spin_thread = std::thread(
        [&](){
            rclcpp::spin(node);
        });
    pthread_setschedparam(spin_thread.native_handle(), SCHED_FIFO,&param);
    
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}