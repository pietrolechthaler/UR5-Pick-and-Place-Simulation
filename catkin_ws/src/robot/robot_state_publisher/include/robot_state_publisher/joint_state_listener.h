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

#ifndef JOINT_STATE_LISTENER_H
#define JOINT_STATE_LISTENER_H

#include <memory>
#include <map>
#include <string>

#include <boost/scoped_ptr.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "robot_state_publisher/robot_state_publisher.h"

namespace robot_state_publisher {

typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;
typedef std::map<std::string, urdf::JointMimicSharedPtr > MimicMap;

class JointStateListener {
public:
  /** Default constructor.
   */
  JointStateListener();

  /** Constructor
   * \param tree The kinematic model of a robot, represented by a KDL Tree
   */
  JointStateListener(const KDL::Tree& tree, const MimicMap& m, const urdf::Model& model = urdf::Model());

  JointStateListener(const std::shared_ptr<RobotStatePublisher>& rsp, const MimicMap& m);


  /// Destructor
  ~JointStateListener();

private:
  std::string getTFPrefix();

protected:
  virtual void callbackJointState(const JointStateConstPtr& state);
  virtual void callbackFixedJoint(const ros::TimerEvent& e);

  ros::Duration publish_interval_;
  std::shared_ptr<RobotStatePublisher> state_publisher_;
  ros::Subscriber joint_state_sub_;
  ros::Timer timer_;
  ros::Time last_callback_time_;
  std::map<std::string, ros::Time> last_publish_time_;
  MimicMap mimic_;
  bool use_tf_static_;
  bool ignore_timestamp_;

};
}

#endif