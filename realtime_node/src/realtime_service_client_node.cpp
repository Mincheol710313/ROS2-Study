#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include <pthread.h>
#include <thread>

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

class MinimalClient : public rclcpp::Node {
public:
    MinimalClient() 
    : Node("minimal_client"), count_(0), loop_count_(1000)
    {   
        this->declare_parameter<int>("loop_count", 1000);
        this->get_parameter("loop_count", loop_count_);
        minimal_client_ = this->create_client<AddTwoInts>("add_two_ints");
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(&MinimalClient::request_result, this)
        );
    }

private:
    void request_result() {
        if (!minimal_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service to be available...");
            return;
        }

        if(this->count_ < this->loop_count_)
        {
            auto request = std::make_shared<AddTwoInts::Request>();
            request->a = 40;
            request->b = this->count_;

            using ServiceResponseFuture = rclcpp::Client<AddTwoInts>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                try {
                    auto result = future.get();
                    RCLCPP_INFO(this->get_logger(), "Result : %" PRId64, result->sum);
                    this->count_++;
                } 
                catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            };

            minimal_client_->async_send_request(request, response_received_callback);
        }
        else{
            rclcpp::shutdown();
        }
    }

    rclcpp::Client<AddTwoInts>::SharedPtr minimal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
    int loop_count_;
};

int main(int argc, char * argv[]){
    struct sched_param param;
    param.sched_priority = 99;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalClient>();
    auto spin_thread = std::thread(
        [&](){
            rclcpp::spin(node);
    });
    pthread_setschedparam(spin_thread.native_handle(), SCHED_FIFO, &param);
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}



