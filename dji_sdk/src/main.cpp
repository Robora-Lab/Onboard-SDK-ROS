/** @file main.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  DJISDKNode::SharedPtr dji_sdk_node = std::make_shared<DJISDKNode>();
  executor.add_node(dji_sdk_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
