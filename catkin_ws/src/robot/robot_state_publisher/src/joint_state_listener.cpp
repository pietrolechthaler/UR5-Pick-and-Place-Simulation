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

#include <algorithm>
#include <map>
#include <string>

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"

using namespace robot_state_publisher;

JointStateListener::JointStateListener() : JointStateListener(KDL::Tree(), MimicMap())
{
}

JointStateListener::JointStateListener(const KDL::Tree& tree, const MimicMap& m, const urdf::Model& model)
  : JointStateListener(std::make_shared<RobotStatePublisher>(tree, model), m)
{
}

JointStateListener::JointStateListener(const std::shared_ptr<RobotStatePublisher>& rsp, const MimicMap& m)
  : state_publisher_(rsp), mimic_(m)
{
  ros::NodeHandle n_tilde("~");
  ros::NodeHandle n;

  // set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency", publish_freq, 50.0);
  // set whether to use the /tf_static latched static transform broadcaster
  n_tilde.param("use_tf_static", use_tf_static_, true);
  // ignore_timestamp_ == true, joins_states messages are accepted, no matter their timestamp
  n_tilde.param("ignore_timestamp", ignore_timestamp_, false);
  // get the tf_prefix parameter from the closest namespace
  publish_interval_ = ros::Duration(1.0/std::max(publish_freq, 1.0));

  // Setting tcpNoNelay tells the subscriber to ask publishers that connect
  // to set TCP_NODELAY on their side. This prevents some joint_state messages
  // from being bundled together, increasing the latency of one of the messages.
  ros::TransportHints transport_hints;
  transport_hints.tcpNoDelay(true);
  // subscribe to joint state
  joint_state_sub_ = n.subscribe("joint_states", 1, &JointStateListener::callbackJointState, this, transport_hints);

  // trigger to publish fixed joints
  // if using static transform broadcaster, this will be a oneshot trigger and only run once
  timer_ = n_tilde.createTimer(publish_interval_, &JointStateListener::callbackFixedJoint, this, use_tf_static_);
}


JointStateListener::~JointStateListener()
{}

std::string JointStateListener::getTFPrefix()
{
  ros::NodeHandle n_tilde("~");
  std::string tf_prefix;

  // get the tf_prefix parameter from the closest namespace
  std::string tf_prefix_key;
  n_tilde.searchParam("tf_prefix", tf_prefix_key);
  n_tilde.param(tf_prefix_key, tf_prefix, std::string(""));

  return tf_prefix;
}

void JointStateListener::callbackFixedJoint(const ros::TimerEvent& e)
{
  (void)e;
  state_publisher_->publishFixedTransforms(getTFPrefix(), use_tf_static_);
}

void JointStateListener::callbackJointState(const JointStateConstPtr& state)
{
  if (state->name.size() != state->position.size()){
    if (state->position.empty()){
      const int throttleSeconds = 300;
      ROS_WARN_THROTTLE(throttleSeconds,
          "Robot state publisher ignored a JointState message about joint(s) "
          "\"%s\"(,...) whose position member was empty. This message will "
          "not reappear for %d seconds.", state->name[0].c_str(),
          throttleSeconds);
    } else {
      ROS_ERROR("Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  ros::Time now = ros::Time::now();
  if (last_callback_time_ > now) {
    // force re-publish of joint transforms
    ROS_WARN("Moved backwards in time (probably because ROS clock was reset), re-publishing joint transforms!");
    last_publish_time_.clear();
  }
  ros::Duration warning_threshold(30.0);
  if ((state->header.stamp + warning_threshold) < now) {
    ROS_WARN_THROTTLE(10, "Received JointState is %f seconds old.", (now - state->header.stamp).toSec());
  }
  last_callback_time_ = now;

  // determine least recently published joint
  ros::Time last_published = now;
  for (size_t i = 0; i < state->name.size(); ++i) {
    ros::Time t = last_publish_time_[state->name[i]];
    last_published = (t < last_published) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  if (ignore_timestamp_ || state->header.stamp >= last_published + publish_interval_) {
    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < state->name.size(); ++i) {
      joint_positions.insert(make_pair(state->name[i], state->position[i]));
    }

    for (MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++) {
      if(joint_positions.find(i->second->joint_name) != joint_positions.end()) {
        double pos = joint_positions[i->second->joint_name] * i->second->multiplier + i->second->offset;
        joint_positions.insert(make_pair(i->first, pos));
      }
    }

    state_publisher_->publishTransforms(joint_positions, state->header.stamp, getTFPrefix());

    // store publish time in joint map
    for (size_t i = 0; i<state->name.size(); ++i) {
      last_publish_time_[state->name[i]] = state->header.stamp;
    }
  }
}