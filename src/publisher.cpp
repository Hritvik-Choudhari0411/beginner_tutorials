/******************************************************************************
  *MIT License

  *Copyright (c) 2022

  *Permission is hereby granted, free of charge, to any person obtaining a copy
  *of this software and associated documentation files (the "Software"), to deal
  *in the Software without restriction, including without limitation the rights
  *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  *copies of the Software, and to permit persons to whom the Software is
  *furnished to do so, subject to the following conditions:

  *The above copyright notice and this permission notice shall be included in
  all *copies or substantial portions of the Software.

  *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  *SOFTWARE.
  ******************************************************************************/
  
/**
 * @file publisher.cpp
 * @author Hritvik Choudhari (hac@umd.edu)
 * @brief Publisher file
 * @version 0.2
 * @date 2022-11-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <beginner_tutorials/srv/string_mod.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

using sharedFuture =
    rclcpp::Client<beginner_tutorials::srv::StringMod>::SharedFuture;

/**
 * @brief MinimalPublisher class, defines the publisher, service client and the
 * associated function
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Parameter Declaration
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Setting callback freq.";
    this->declare_parameter("freq", 1.0, param_desc);
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    RCLCPP_DEBUG(this->get_logger(),
                 "Parameter freq declared, set to default 1.0 hz");

    // Creating a subscriber for the parameter
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto parameterCallbackPtr =
        std::bind(&MinimalPublisher::param_cb, this, _1);
    param_handle_ =
        param_subscriber_->add_parameter_callback("freq", parameterCallbackPtr);

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    RCLCPP_DEBUG(this->get_logger(), "Publisher is Created");
    auto delta = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        delta, std::bind(&MinimalPublisher::timer_cb, this));

    client =
        this->create_client<beginner_tutorials::srv::StringMod>("modify_msg");
    RCLCPP_DEBUG(this->get_logger(), "Client created");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Interrupted");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Service unavailable, waiting for server to start");
    }
  }

 private:
  std::string Message;
  rclcpp::Client<beginner_tutorials::srv::StringMod>::SharedPtr client;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> param_handle_;

  /**
   * @brief Timer callback function, sets the message data and publishes the
   * message and also calls the service at every 10 counts
   */
  void timer_cb() {
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Node setup");
    auto message = std_msgs::msg::String();
    message.data = "Message published ID " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Outgoing signal : '%s'",
                message.data.c_str());
    publisher_->publish(message);
    if (count_ % 10 == 0) {
      service_cb();
    }
    auto steady_clock = rclcpp::Clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), steady_clock, 10000,
                                 "Node running successfully");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  /**
   * @brief Service callback function, defines the service parameters and calls
   * the response
   * @return int
   */
  int service_cb() {
    auto request =
        std::make_shared<beginner_tutorials::srv::StringMod::Request>();
    request->a = "Message 1";
    request->b = " Message 2";
    RCLCPP_INFO(this->get_logger(), "Service called to Modify string");
    auto callbackPtr = std::bind(&MinimalPublisher::response_cb, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }

  /**
   * @brief Response callback function, calls the response for the service_cb
   * function
   * @param future
   */
  void response_cb(sharedFuture future) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "Got String: %s", future.get()->c.c_str());
    Message = future.get()->c.c_str();
  }

  /**
   * @brief Parameter callback function, assigns the updated value of the
   * parameter
   * @param param
   */
  void param_cb(const rclcpp::Parameter &param) {
    RCLCPP_INFO(this->get_logger(),
                "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(),
                param.as_double());
    RCLCPP_WARN(this->get_logger(), "Message frequency changed");

    RCLCPP_FATAL_EXPRESSION(this->get_logger(), param.as_double() == 0.0,
                            "Frequency is set to zero, zero division error");
    if (param.as_double() == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Frequency has not been changed.");
    } else {
      auto delta = std::chrono::milliseconds(
          static_cast<int>((1000 / param.as_double())));
      timer_ = this->create_wall_timer(
          delta, std::bind(&MinimalPublisher::timer_cb, this));
    }
  }
};

/**
 * @brief Main function
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
