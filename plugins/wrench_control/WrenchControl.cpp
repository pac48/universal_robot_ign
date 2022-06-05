/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

//#include "ignition/gazebo/components/AngularVelocityCmd.hh"
//#include "ignition/gazebo/components/LinearVelocityCmd.hh"
#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "WrenchControl.h"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::WrenchControlPrivate
{
  /// \brief Callback for model velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdWrench(const ignition::msgs::Wrench &_msg);

  /// \brief Callback for link velocity subscription
  /// \param[in] _msg Velocity message
//  public: void OnLinkCmdVel(const ignition::msgs::Twist &_msg,
//    const ignition::transport::MessageInfo &_info);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
//  public: void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
//    const ignition::gazebo::EntityComponentManager &_ecm);
//
//  /// \brief Update link velocity.
//  /// \param[in] _info System update information.
//  /// \param[in] _ecm The EntityComponentManager of the given simulation
//  /// instance.
//  public: void UpdateLinkVelocity(const ignition::gazebo::UpdateInfo &_info,
//    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief A mutex to protect the model velocity command.
  public: std::mutex mutex;

  /// \brief Link names
  public: std::string linkName;

  /// \brief Link entities in a model
  public: Entity link;

  public: ignition::msgs::Wrench commanded_wrench;

  /// \brief Linear velocities of links
  public: std::unordered_map<std::string, ignition::msgs::Wrench> wrench;

};

//////////////////////////////////////////////////
WrenchControl::WrenchControl()
  : dataPtr(std::make_unique<WrenchControlPrivate>())
{
}

//////////////////////////////////////////////////
void WrenchControl::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "WrenchControl plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  if (!ptr->HasElement("link_name"))
  {
        ignerr << "No link_name tag found" << std::endl;
        return;
  }

  sdf::ElementPtr sdfElem = ptr->GetElement("link_name");
  this->dataPtr->linkName = sdfElem->Get<std::string>();

    std::string topic = "/model/" + this->dataPtr->model.Name(_ecm) +"/link/" + this->dataPtr->linkName  + "/cmd_wrench";

    this->dataPtr->node.Subscribe(topic, &WrenchControlPrivate::OnCmdWrench, this->dataPtr.get());
    ignmsg << "WrenchControl subscribing to wrench messages on ["
           << topic << "]"
           << std::endl;
}

//////////////////////////////////////////////////
void WrenchControl::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("WrenchControl::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;


  auto modelName = this->dataPtr->model.Name(_ecm);

  if (this->dataPtr->linkName.empty())
    return;

        Entity link = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
        if (link != kNullEntity)
          this->dataPtr->link =  link;
        else
        {
          ignwarn << "Failed to find link [" << this->dataPtr->linkName
                << "] for model [" << modelName << "]" << std::endl;
        }

//   update link wrench

    auto linkLinearWrenchComp =_ecm.Component<components::ExternalWorldWrenchCmd>(this->dataPtr->link);
    if (!linkLinearWrenchComp)
    {
        _ecm.CreateComponent(this->dataPtr->link,
                             components::ExternalWorldWrenchCmd({this->commanded_wrench}));
    }
    else
    {
        *linkLinearWrenchComp = components::ExternalWorldWrenchCmd(this->commanded_wrench);
    }

}

//////////////////////////////////////////////////
void WrenchControl::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("WrenchControl::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
this->commanded_wrench = this->dataPtr->commanded_wrench;
}




//////////////////////////////////////////////////
void WrenchControlPrivate::OnCmdWrench(const msgs::Wrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->commanded_wrench = _msg;
}

//////////////////////////////////////////////////
//void WrenchControlPrivate::OnLinkCmdVel(const msgs::Twist &_msg,
//                                      const transport::MessageInfo &_info)
//{
//  std::lock_guard<std::mutex> lock(this->mutex);
//  for (const auto &linkName : this->linkName)
//  {
//    if (_info.Topic().find("/" + linkName + "/cmd_vel") != std::string::npos)
//    {
////      this->linkVels.insert({linkName, _msg});
//      break;
//    }
//  }
//}

IGNITION_ADD_PLUGIN(WrenchControl,
                    ignition::gazebo::System,
                    WrenchControl::ISystemConfigure,
                    WrenchControl::ISystemPreUpdate,
                    WrenchControl::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WrenchControl,
                          "ignition::gazebo::systems::WrenchControl")
