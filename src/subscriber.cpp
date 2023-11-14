/******************************************************************************
  *MIT License

  *Copyright (c) 2022

  *Permission is hereby granted, free of charge, to any person obtaining a copy
  *of this software and associated documentation files (the "Software"), to deal
  *in the Software without restriction, including without limitation the rights
  *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  *copies of the Software, and to permit persons to whom the Software is
  *furnished to do so, subject to the following conditions:

  *The above copyright notice and this permission notice shall be included in all
  *copies or substantial portions of the Software.

  *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  *SOFTWARE.
  ******************************************************************************/
/**
 * @file subscriber.cpp
 * @author Hritvik Choudhari (hac@umd.edu)
 * @brief Subscriber file
 * @version 0.12
 * @date 2022-11-14
 *
 * @copyright MIT Copyright (c) 2022
 *
 */
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief MinimalSubscriber class
 *
 * Subscribes to a topic and displays incoming signals.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function for the subscribed topic
   *
   * @param msg The incoming message from the topic
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "Incoming signal : '%s'", msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Main function
 *
 * Initializes the ROS 2 node, spins it, and shuts it down.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Program exit code
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}