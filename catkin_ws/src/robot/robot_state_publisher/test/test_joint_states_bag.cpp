/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Wim Meeussen */

#include <string>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/thread/thread.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher/joint_state_listener.h"

using namespace ros;
using namespace tf2_ros;
using namespace robot_state_publisher;


int g_argc;
char** g_argv;

#define EPS 0.01

class TestPublisher : public testing::Test
{
public:
  JointStateListener* publisher;

protected:
  /// constructor
  TestPublisher()
  {}

  /// Destructor
  ~TestPublisher()
  {}
};


TEST_F(TestPublisher, test)
{
  ROS_INFO("Creating tf listener");
  Buffer buffer;
  TransformListener tf(buffer);

  ROS_INFO("Waiting for bag to complete");
  Duration(15.0).sleep();

  ASSERT_TRUE(buffer.canTransform("base_link", "torso_lift_link", Time()));
  ASSERT_TRUE(buffer.canTransform("base_link", "r_gripper_palm_link", Time()));
  ASSERT_TRUE(buffer.canTransform("base_link", "r_gripper_palm_link", Time()));
  ASSERT_TRUE(buffer.canTransform("l_gripper_palm_link", "r_gripper_palm_link", Time()));
  ASSERT_TRUE(buffer.canTransform("l_gripper_palm_link", "fl_caster_r_wheel_link", Time()));
  ASSERT_FALSE(buffer.canTransform("base_link", "wim_link", Time()));

  geometry_msgs::TransformStamped t = buffer.lookupTransform("base_link", "r_gripper_palm_link", Time());
  EXPECT_NEAR(t.transform.translation.x, 0.769198, EPS);
  EXPECT_NEAR(t.transform.translation.y, -0.188800, EPS);
  EXPECT_NEAR(t.transform.translation.z, 0.764914, EPS);

  t = buffer.lookupTransform("l_gripper_palm_link", "r_gripper_palm_link", Time());
  EXPECT_NEAR(t.transform.translation.x, 0.000384222, EPS);
  EXPECT_NEAR(t.transform.translation.y, -0.376021, EPS);
  EXPECT_NEAR(t.transform.translation.z, -1.0858e-05, EPS); SUCCEED();
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_robot_state_publisher");

  g_argc = argc;
  g_argv = argv;
  int res = RUN_ALL_TESTS();

  return res;
}