#include <boost/thread/recursive_mutex.hpp>

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include "gazebo_ros_link_attacher.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"

#include "gazebo_ros_link_attacher/SetStatic.h"
#include "gazebo_ros_link_attacher/SetStaticRequest.h"
#include "gazebo_ros_link_attacher/SetStaticResponse.h"

#include <ignition/math/Pose3.hh>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

  // Constructor
  GazeboRosLinkAttacher::GazeboRosLinkAttacher() :
    nh_("link_attacher_node")
  {
  }


  // Destructor
  GazeboRosLinkAttacher::~GazeboRosLinkAttacher()
  {
  }

  void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    this->world = _world;
    this->physics = this->world->Physics();
    this->physics_mutex = this->physics->GetPhysicsUpdateMutex();

    this->setstatic_service_ = this->nh_.advertiseService("setstatic", &GazeboRosLinkAttacher::setstatic_callback, this);
    ROS_INFO_STREAM("SetStatic service at: " << this->nh_.resolveName("setstatic"));

    this->attach_service_ = this->nh_.advertiseService("attach", &GazeboRosLinkAttacher::attach_callback, this);
    ROS_INFO_STREAM("Attach service at: " << this->nh_.resolveName("attach"));
    this->detach_service_ = this->nh_.advertiseService("detach", &GazeboRosLinkAttacher::detach_callback, this);
    ROS_INFO_STREAM("Detach service at: " << this->nh_.resolveName("detach"));
    ROS_INFO("Link attacher node initialized.");
  }

  bool GazeboRosLinkAttacher::attach(std::string model1, std::string link1,
                                     std::string model2, std::string link2)
  {

    // look for any previous instance of the joint first.
    // if we try to create a joint in between two links
    // more than once (even deleting any reference to the first one)
    // gazebo hangs/crashes
    fixedJoint j;
    if(this->getJoint(model1, link1, model2, link2, j)){
        ROS_INFO_STREAM("Joint already existed, reusing it.");
        j.joint->Attach(j.l1, j.l2);
        return true;
    }
    else{
        ROS_INFO_STREAM("Creating new joint.");
    }

    j.model1 = model1;
    j.link1 = link1;
    j.model2 = model2;
    j.link2 = link2;

    ROS_DEBUG_STREAM("Casting into ModelPtr");
    physics::ModelPtr m1 = this->world->ModelByName(model1);
    j.m1 = m1;
    physics::ModelPtr m2 = this->world->ModelByName(model2);
    j.m2 = m2;

    if (m1 == NULL){
      ROS_ERROR_STREAM(m1 << " model was not found");
      return false;
    }
    if (m2 == NULL){
      ROS_ERROR_STREAM(m2 << " model was not found");
      return false;
    }
    

    ROS_DEBUG_STREAM("Getting link: '" << link1 << "' from model: '" << model1 << "'");
    physics::LinkPtr l1 = m1->GetLink(link1);
    if (l1 == NULL){
      ROS_ERROR_STREAM(link1 << " link was not found");
      return false;
    }
    if (l1->GetInertial() == NULL){
        ROS_ERROR_STREAM("link1 inertia is NULL!");
    }
    else
        ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
    j.l1 = l1;
    ROS_DEBUG_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
    physics::LinkPtr l2 = m2->GetLink(link2);
    if (l2 == NULL){
      ROS_ERROR_STREAM(link2 << " link was not found");
      return false;
    }
    if (l2->GetInertial() == NULL){
        ROS_ERROR_STREAM("link2 inertia is NULL!");
    }
    else
        ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
    j.l2 = l2;

    ROS_DEBUG_STREAM("Links are: "  << l1->GetName() << " and " << l2->GetName());

    ROS_DEBUG_STREAM("Creating revolute joint on model: '" << model1 << "'");
    j.joint = this->physics->CreateJoint("fixed", m1);
    
    this->joints.push_back(j);

    ROS_DEBUG_STREAM("Attach");
    j.joint->Attach(l1, l2);
    ROS_DEBUG_STREAM("Loading links");
    j.joint->Load(l1, l2, ignition::math::Pose3d());
    ROS_DEBUG_STREAM("SetModel");
    j.joint->SetModel(m2);

    /*
     * If SetModel is not done we get:
     * ***** Internal Program Error - assertion (this->GetParentModel() != __null)
     failed in void gazebo::physics::Entity::PublishPose():
     /tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
     An entity without a parent model should not happen

     * If SetModel is given the same model than CreateJoint given
     * Gazebo crashes with
     * ***** Internal Program Error - assertion (self->inertial != __null)
     failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
     /tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
     */

    ROS_DEBUG_STREAM("SetHightstop");
    j.joint->SetUpperLimit(0, 0);
    ROS_DEBUG_STREAM("SetLowStop");
    j.joint->SetLowerLimit(0, 0);
    ROS_DEBUG_STREAM("Init");
    j.joint->Init();
    ROS_INFO_STREAM("Attach finished.");

    return true;
  }
  
  bool GazeboRosLinkAttacher::setStatic(std::string model, std::string link, bool set_static)
  {
    physics::ModelPtr m = this->world->ModelByName(model);

    ROS_DEBUG_STREAM("Getting BasePtr of " << model);
    if (m == NULL){
      ROS_ERROR_STREAM(model << " model was not found");
      return false;
    }

    ROS_DEBUG_STREAM("Getting link: '" << link << "' from model: '" << model << "'");
    physics::LinkPtr l = m->GetLink(link);
    if (l == NULL){
      ROS_ERROR_STREAM(link << " link was not found");
      return false;
    }

    ROS_DEBUG_STREAM("Setting is_static for link " << link << " of model " << model << " to " << std::boolalpha << set_static);
    m->SetCollideMode("none");
    m->SetGravityMode(false);
    m->ResetPhysicsStates();

    return true;
  }

  bool GazeboRosLinkAttacher::detach(std::string model1, std::string link1,
                                     std::string model2, std::string link2)
  {
    // search for the instance of joint and do detach
    fixedJoint j;
    if(this->getJoint(model1, link1, model2, link2, j)){
        boost::recursive_mutex::scoped_lock lock(*this->physics_mutex);
        j.joint->Detach();
        return true;
    }

    return false;
  }

  bool GazeboRosLinkAttacher::getJoint(std::string model1, std::string link1,
                                       std::string model2, std::string link2,
                                       fixedJoint &joint){
    fixedJoint j;
    for(std::vector<fixedJoint>::iterator it = this->joints.begin(); it != this->joints.end(); ++it){
        j = *it;
        if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0)
                && (j.link1.compare(link1) == 0) && (j.link2.compare(link2) == 0)){
            joint = j;
            return true;
        }
    }
    return false;

  }

  bool GazeboRosLinkAttacher::setstatic_callback(gazebo_ros_link_attacher::SetStatic::Request &req,
                                              gazebo_ros_link_attacher::SetStatic::Response &res)
  {
    ROS_INFO_STREAM("Received request to change is_static of model: " << req.model_name);
    if (! this->setStatic(req.model_name, req.link_name, req.set_static)){
      ROS_ERROR_STREAM("Could not change is_static property.");
      res.ok = false;
    }
    else{
      ROS_INFO_STREAM("Model property is_static changed to " << std::boolalpha << req.set_static);
      res.ok = true;
    }
    return true;

  }

  bool GazeboRosLinkAttacher::attach_callback(gazebo_ros_link_attacher::Attach::Request &req,
                                              gazebo_ros_link_attacher::Attach::Response &res)
  {
    ROS_INFO_STREAM("Received request to attach model: '" << req.model_name_1
                    << "' using link: '" << req.link_name_1 << "' with model: '"
                    << req.model_name_2 << "' using link: '" <<  req.link_name_2 << "'");
    if (! this->attach(req.model_name_1, req.link_name_1,
                       req.model_name_2, req.link_name_2)){
      ROS_ERROR_STREAM("Could not make the attach.");
      res.ok = false;
    }
    else{
      ROS_INFO_STREAM("Attach was succesful");
      res.ok = true;
    }
    return true;

  }

  bool GazeboRosLinkAttacher::detach_callback(gazebo_ros_link_attacher::Attach::Request &req,
                                              gazebo_ros_link_attacher::Attach::Response &res){
      ROS_INFO_STREAM("Received request to detach model: '" << req.model_name_1
                      << "' using link: '" << req.link_name_1 << "' with model: '"
                      << req.model_name_2 << "' using link: '" <<  req.link_name_2 << "'");
      if (! this->detach(req.model_name_1, req.link_name_1,
                         req.model_name_2, req.link_name_2)){
        ROS_ERROR_STREAM("Could not make the detach.");
        res.ok = false;
      }
      else{
        ROS_INFO_STREAM("Detach was succesful");
        res.ok = true;
      }
      return true;
  }

}
