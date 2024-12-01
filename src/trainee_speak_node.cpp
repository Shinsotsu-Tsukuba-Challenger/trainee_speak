// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "trainee_speak/trainee_speak.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<trainee_speak::RaspicatSpeak2>(
      "trainee_speak_node", argv[1]);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}