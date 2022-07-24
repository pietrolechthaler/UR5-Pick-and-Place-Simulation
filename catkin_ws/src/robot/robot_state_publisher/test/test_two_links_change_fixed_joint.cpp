// Copyright (c) 2021, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

static constexpr double EPS = 0.01;

TEST(test_publisher, test_two_joints)
{
  auto node = rclcpp::Node::make_shared("rsp_test_two_links_change_fixed_joint");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer, node, true);
  unsigned int i;

  for (i = 0; i < 100 && !buffer.canTransform("link1", "link2", rclcpp::Time()); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(buffer.canTransform("link1", "link2", rclcpp::Time()));
  ASSERT_FALSE(buffer.canTransform("base_link", "wim_link", rclcpp::Time()));

  geometry_msgs::msg::TransformStamped t = buffer.lookupTransform("link1", "link2", rclcpp::Time());
  EXPECT_NEAR(t.transform.translation.x, 5.0, EPS);
  EXPECT_NEAR(t.transform.translation.y, 0.0, EPS);
  EXPECT_NEAR(t.transform.translation.z, 0.0, EPS);

  // OK, now publish a new URDF to the parameter and ensure that the link has been updated
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    node,
    "robot_state_publisher");
  for (i = 0; i < 100 && !parameters_client->wait_for_service(std::chrono::milliseconds(100));
    ++i)
  {
    ASSERT_TRUE(rclcpp::ok());
  }
  ASSERT_LT(i, 100u);

  std::string new_robot_description(
    "<robot name=\"test_robot_fixed_joint\">"
    "  <link name=\"link1\" />"
    "  <link name=\"link2\" />"
    "  <joint name=\"joint1\" type=\"fixed\">"
    "    <parent link=\"link1\"/>"
    "    <child link=\"link2\"/>"
    "    <origin xyz=\"10 0 0\" rpy=\"0 0 1.57\" />"
    "  </joint>"
    "</robot>");
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_result =
    parameters_client->set_parameters(
  {
    rclcpp::Parameter("robot_description", new_robot_description)
  }, std::chrono::milliseconds(500));
  ASSERT_EQ(set_parameters_result.size(), 1u);
  ASSERT_TRUE(set_parameters_result[0].successful);

  for (i = 0; i < 100; ++i) {
    if (buffer.canTransform("link1", "link2", rclcpp::Time())) {
      t = buffer.lookupTransform("link1", "link2", rclcpp::Time());
      if (fabs(t.transform.translation.x - 10.0) <= EPS) {
        ASSERT_NEAR(t.transform.translation.x, 10.0, EPS);
        ASSERT_NEAR(t.transform.translation.y, 0.0, EPS);
        ASSERT_NEAR(t.transform.translation.z, 0.0, EPS);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_LT(i, 100u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int res = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return res;
}
