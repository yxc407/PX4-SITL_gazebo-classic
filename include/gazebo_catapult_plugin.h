/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Catapult Plugin
 *
 * This plugin simulates a catapult / handlaunch for fixed wing vehicles
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#ifndef _GAZEBO_CATAPULT_PLUGIN_HH_
#define _GAZEBO_CATAPULT_PLUGIN_HH_

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include "CommandMotorSpeed.pb.h"


#include <Odometry.pb.h>

namespace gazebo
{

typedef const boost::shared_ptr<const msgs::Int> triggerMsg;

enum LaunchStatus {
    VEHICLE_STANDBY,
    VEHICLE_INLAUNCH,
    VEHICLE_LAUNCHED
};

class GAZEBO_VISIBLE CatapultPlugin : public ModelPlugin
{
public:
  CatapultPlugin();
  virtual ~CatapultPlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  void SwitchCallback(triggerMsg &msg);

  std::string namespace_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;

  event::ConnectionPtr _updateConnection;

  LaunchStatus launch_status_ = VEHICLE_STANDBY;
  common::Time trigger_time_;

  double launch_duration_ = 0.01;
  double force_magnitude_ = 1.0;
  ignition::math::Vector3d direction_ = ignition::math::Vector3d(0.0, 1.0, 0.0);

  bool switch_ = false;
  bool triggered_ = false;

  std::string trigger_sub_topic_ = "/catapult_trigger";

  transport::NodePtr node_handle_;
  transport::SubscriberPtr trigger_sub_;

};     // class GAZEBO_VISIBLE CatapultPlugin
}      // namespace gazebo
#endif // _GAZEBO_CATAPULT_PLUGIN_HH_
