#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

#include <pthread.h>
#include <thread>
#include <fstream>

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;
using Clock = std::chrono::steady_clock;

class MinimalServer : public rclcpp::Node {
public:
  MinimalServer()
  : Node("minimal_server"), last_request_time_(Clock::now())
  {
    this->declare_parameter<std::string>("log_file", "Service_Realtime_Result_1000.txt");
    this->get_parameter("log_file", log_file_);
    
    log_stream_.open(log_file_);
    if(!log_stream_.is_open()){
      RCLCPP_ERROR(this->get_logger(), "Falied to open log file : %s", log_file_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Success to open log file : %s", log_file_.c_str());
      log_stream_ << "Iteration Result Response-Time Duration[ns]\n";
    }

    minimal_server_ = this->create_service<AddTwoInts>(
      "add_two_ints",
      std::bind(&MinimalServer::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  ~MinimalServer() {
    if(log_stream_.is_open()){
      log_stream_.close();
    }
  }

  void handle_service(
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
  {
    auto now = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_request_time_);
    response->sum = request->a + request->b;
    auto response_time = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - now);

    this->log_stream_ << request->b << " " << response->sum << " " << response_time.count() <<" "<< duration.count() << std::endl;

    last_request_time_ = now;
  }

private:
  std::string log_file_;
  std::ofstream log_stream_;
  rclcpp::Service<AddTwoInts>::SharedPtr minimal_server_;
  std::chrono::time_point<Clock> last_request_time_;

};

int main(int argc, char ** argv)
{
  struct sched_param param;
   param.sched_priority = 99;  
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalServer>();

  auto spin_thread = std::thread(
    [&](){
        rclcpp::spin(node);
    });
  pthread_setschedparam(spin_thread.native_handle(), SCHED_FIFO, &param);
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}