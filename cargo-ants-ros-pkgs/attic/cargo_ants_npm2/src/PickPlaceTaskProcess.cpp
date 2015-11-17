/* Cargo-ANTs software prototype.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "PickPlaceTaskProcess.hpp"
#include <npm2/Simulator.hpp>
#include <npm2/Object.hpp>

#include "cargo_ants_msgs/VehicleInfo.h"


namespace cargo_ants_npm2 {
  
  
  PickPlaceTaskProcess::
  PickPlaceTaskProcess (string const & name)
    : Process (name),
      container_manager_ (0),
      control_ (0),
      container_ (0),
      msg_queue_size_ (10),
      task_topic_ ("task"),
      vehicle_info_topic_ ("vehicle_info")
  {
    reflectSlot ("container_manager", &container_manager_);
    reflectSlot ("control", &control_);
    reflectParameter ("msg_queue_size", &msg_queue_size_);
    reflectParameter ("task_topic", &task_topic_);
    reflectParameter ("vehicle_info_topic", &vehicle_info_topic_);
  }
  
  
  Process::state_t PickPlaceTaskProcess::
  init (ostream & erros)
  {
    if ( ! container_manager_) {
      erros << "PickPlaceTaskProcess needs a ContainerManager instance\n";
      return FAILED;
    }
    if ( ! control_) {
      erros << "PickPlaceTaskProcess needs a KinematicControl instance\n";
      return FAILED;
    }
    if ( ! ros::ok()) {
      erros << "PickPlaceTaskProcess needs ROS to be up and running\n";
      return FAILED;
    }
    
    ros::NodeHandle node;
    task_sub_ = node.subscribe (task_topic_, msg_queue_size_,
				&PickPlaceTaskProcess::taskCallback, this);
    vehicle_info_pub_ = node.advertise <cargo_ants_msgs::VehicleInfo> (vehicle_info_topic_, 1);
    vehicle_info_.vehicle = control_->drive_->getParent()->name;
    vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::IDLE;
    control_->enable (false);
    
    return RUNNING;
  }
  
  
  Process::state_t PickPlaceTaskProcess::
  run (double timestep, ostream & erros)
  {
    if ( ! ros::ok()) {
      erros << "PickPlaceTaskProcess needs ROS to be up and running\n";
      return FAILED;
    }
    ros::spinOnce();
    
    sfl::Frame const & pose (control_->drive_->getParent()->getGlobal());
    
    switch (vehicle_info_.mode) {
      
    case cargo_ants_msgs::VehicleInfo::PICKING:
      if (pickup_.Reached (pose, true)) {
	if (container_manager_->grabContainer (vehicle_info_.vehicle, container_->name)) {
	  vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::PLACING;
	  vehicle_info_.container = container_->name;
	  control_->setGoal (placedown_);
	}
	else {
	  ROS_ERROR ("vehicle %s failed to grab container %s",
		     vehicle_info_.vehicle.c_str(), container_->name.c_str());
	  vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::ERROR;
	  vehicle_info_.container = "";
	  control_->enable (false);
	}
      }
      break;
    
    case cargo_ants_msgs::VehicleInfo::PLACING:
      if (placedown_.Reached (pose, true)) {
	if (container_manager_->releaseContainer (vehicle_info_.vehicle, container_->name)) {
	  vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::IDLE;
	  vehicle_info_.container = "";
	}
	else {
	  ROS_ERROR ("vehicle %s failed to release container %s",
		     vehicle_info_.vehicle.c_str(), container_->name.c_str());
	  vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::ERROR;
	  vehicle_info_.container = "";
	}
	control_->enable (false);
      }
      break;
      
    case cargo_ants_msgs::VehicleInfo::IDLE:
      if ( ! tasks_.empty()) {
	container_ = Object::registry.find (tasks_.front().container);
	if ( ! container_) {
	  ROS_ERROR ("vehicle %s failed to find container %s",
		     vehicle_info_.vehicle.c_str(), tasks_.front().container.c_str());
	  vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::ERROR;
	  vehicle_info_.container = "";
	}
	else {
	  vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::PICKING;
	  vehicle_info_.container = container_->name;
	  pickup_.Set (tasks_.front().pickup.gx,
		       tasks_.front().pickup.gy,
		       tasks_.front().pickup.gth,
		       tasks_.front().pickup.dr,
		       tasks_.front().pickup.dth);
	  placedown_.Set (tasks_.front().placedown.gx,
			  tasks_.front().placedown.gy,
			  tasks_.front().placedown.gth,
			  tasks_.front().placedown.dr,
			  tasks_.front().placedown.dth);
	  control_->setGoal (pickup_);
	  control_->enable (true);
	}
	tasks_.pop();
      }
      break;
      
    case cargo_ants_msgs::VehicleInfo::ERROR:
      // there should be some kind of reset message to get us out of error conditions
      break;
      
    default:
      ROS_ERROR ("vehicle %s unhandled mode %d",
		 vehicle_info_.vehicle.c_str(), vehicle_info_.mode);
      vehicle_info_.mode = cargo_ants_msgs::VehicleInfo::ERROR;
      vehicle_info_.container = "";
    }
    
    vehicle_info_.vehicle_px = pose.X();
    vehicle_info_.vehicle_py = pose.Y();
    vehicle_info_.vehicle_pth = pose.Theta();
    
    vehicle_info_pub_.publish (vehicle_info_);
    
    return RUNNING;
  }
  
  
  void PickPlaceTaskProcess::
  taskCallback (cargo_ants_msgs::Task::ConstPtr msg)
  {
    if (msg->vehicle == vehicle_info_.vehicle) {
      tasks_.push (*msg);
    }
  }
  
}
