/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#include <gtest/gtest.h>

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher/joint_state_listener.h"
#include "robot_state_publisher/robot_state_publisher.h"

namespace robot_state_publisher_test
{
class AccessibleJointStateListener : public robot_state_publisher::JointStateListener
{
public:
  AccessibleJointStateListener(
    const KDL::Tree& tree, const robot_state_publisher::MimicMap& m, const urdf::Model& model) :
      robot_state_publisher::JointStateListener(tree, m, model)
  {
  }

  bool usingTfStatic() const {
    return use_tf_static_;
  }
};

class AccessibleRobotStatePublisher : public robot_state_publisher::RobotStatePublisher
{
public:

  AccessibleRobotStatePublisher(const KDL::Tree& tree, const urdf::Model& model) :
    robot_state_publisher::RobotStatePublisher(tree, model)
  {
  }

  const urdf::Model & getModel() const {
    return model_;
  }
};
}  // robot_state_publisher_test 

TEST(TestRobotStatePubSubclass, robot_state_pub_subclass)
{
  urdf::Model model;
  model.initParam("robot_description");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)){
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    FAIL();
  }

  robot_state_publisher::MimicMap mimic;

  for(std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++){
    if(i->second->mimic){
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }

  robot_state_publisher_test::AccessibleRobotStatePublisher state_pub(tree, model);

  EXPECT_EQ(model.name_, state_pub.getModel().name_);
  EXPECT_EQ(model.root_link_, state_pub.getModel().root_link_);

  robot_state_publisher_test::AccessibleJointStateListener state_listener(tree, mimic, model);
  EXPECT_TRUE(state_listener.usingTfStatic());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_subclass");
  testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS();

  return res;
}