/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Open Source Robotics Foundation, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <tf2_msgs/TFMessage.h>

#define EPS 0.01

class TestFramesAndSlashes : public testing::Test
{
protected:
  TestFramesAndSlashes()
  {
    tf_static_sub_ = n_.subscribe(
      "/tf_static", 1, &TestFramesAndSlashes::tf_static_callback, this);
  }

  ~TestFramesAndSlashes() = default;

  void
  tf_static_callback(const tf2_msgs::TFMessage::ConstPtr msg)
  {
    transforms_received_ = *msg;
    // avoid writing another message while test is reading the one received
    tf_static_sub_.shutdown();
    got_transforms_ = true;
  }

  ros::NodeHandle n_;
  ros::Subscriber tf_sub_;
  ros::Subscriber tf_static_sub_;
  tf2_msgs::TFMessage transforms_received_;
  bool got_transforms_ = false;
};

TEST_F(TestFramesAndSlashes, test)
{
  std::vector<std::string> expected_links(13);

  {
    ros::NodeHandle n_tilde;
    std::string robot_description;
    ASSERT_TRUE(n_tilde.getParam("robot_description", robot_description));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_1", expected_links[0]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_2", expected_links[1]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_3", expected_links[2]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_4", expected_links[3]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_5", expected_links[4]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_6", expected_links[5]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_7", expected_links[6]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_8", expected_links[7]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_9", expected_links[8]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_10", expected_links[9]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_11", expected_links[10]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_12", expected_links[11]));
    ASSERT_TRUE(n_tilde.getParam("expected_name_link_13", expected_links[12]));
  }

  for (int i = 0; i < 100 && !got_transforms_; ++i) {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  ASSERT_TRUE(got_transforms_);

  for (size_t n = 1; n < expected_links.size(); ++n) {
    const std::string & link_1 = expected_links.at(0);
    const std::string & link_n = expected_links.at(n);

    bool found_transform_ = false;
    geometry_msgs::Transform transform;
    for (const geometry_msgs::TransformStamped & tf_stamped : transforms_received_.transforms) {
      if (tf_stamped.header.frame_id == link_1 &&
          tf_stamped.child_frame_id == link_n)
      {
        transform = tf_stamped.transform;
        found_transform_ = true;
      }
    }

    EXPECT_TRUE(found_transform_) << "expected " << link_1 << " and " << link_n;
    if (found_transform_) {
      EXPECT_NEAR(transform.translation.x, n + 1, EPS) << "frames " << link_1 << " and " << link_n;
    }
  }

  // Print transforms received for easier debugging
  EXPECT_FALSE(testing::Test::HasFailure()) << transforms_received_;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_frames_and_slashes");

  int res = RUN_ALL_TESTS();

  return res;
}