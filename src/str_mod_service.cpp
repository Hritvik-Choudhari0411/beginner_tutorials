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
 * @file str_mod_service.cpp
 * @author Hritvik Choudhari (hac@umd.edu)
 * @brief Service file
 * @version 0.1
 * @date 2022-11-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <beginner_tutorials/srv/string_mod.hpp>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using StringMod = beginner_tutorials::srv::StringMod;

/**
 * @brief Function to process the request and concatenate two strings.
 *
 * This function takes a request with two strings (a and b) and concatenates
 * them, storing the result in the response string (c).
 *
 * @param request The request containing two strings (a and b)
 * @param response The response containing the concatenated string (c)
 */
void add(const std::shared_ptr<StringMod::Request> request,
         std::shared_ptr<StringMod::Response> response) {
  response->c = request->a + " & " + request->b + " concatenated.";
}

/**
 * @brief Main function to initialize and spin the service node.
 *
 * Initializes the ROS 2 node, creates a service for string modification, and
 * spins the node to handle incoming service requests.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Program exit code
 */
int main(int argc, char **argv) {
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("modify_msg_server");

  // Create a service for string modification
  rclcpp::Service<StringMod>::SharedPtr service =
      node->create_service<StringMod>("modify_msg", &add);

  // Log an informational message indicating that the service is ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modifying message");

  // Spin the node to handle incoming service requests
  rclcpp::spin(node);

  // Shutdown the ROS 2 node
  rclcpp::shutdown();
  // Return program exit code
  return 0;
}
