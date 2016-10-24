#ifndef _GAZEBO_VEL_PLUGIN_HH_
#define _GAZEBO_VEL_PLUGIN_HH_

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string>
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class Vel_plugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: Vel_plugin();

    /// \brief Destructor.
    public: virtual ~Vel_plugin();

    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate(const common::UpdateInfo & /*_info*/);
    
    private: physics::ModelPtr model;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
    public : ros::Publisher pub, pub2, pub3;
  };
}
#endif