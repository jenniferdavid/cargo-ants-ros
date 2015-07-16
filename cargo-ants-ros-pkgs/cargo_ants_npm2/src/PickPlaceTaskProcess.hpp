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

#ifndef CARGO_ANTS_NPM2_PICK_PLACE_TASK_PROCESS_HPP
#define CARGO_ANTS_NPM2_PICK_PLACE_TASK_PROCESS_HPP

#include <npm2/Process.hpp>
#include <npm2/KinematicControl.hpp>
#include <sfl/api/Goal.hpp>
#include <queue>

#include "ContainerManager.hpp"
#include "ros/ros.h"
#include "cargo_ants_msgs/VehicleInfo.h"
#include "cargo_ants_msgs/Task.h"


namespace cargo_ants_npm2 {

  using namespace npm2;
  
  
  class PickPlaceTaskProcess
    : public Process
  {
  public:
    explicit PickPlaceTaskProcess (string const & name);
    
  protected:
    virtual state_t init (ostream & erros);
    virtual state_t run (double timestep, ostream & erros);
    
    void taskCallback (cargo_ants_msgs::Task::ConstPtr msg);
    
    ContainerManager * container_manager_;
    KinematicControl * control_;
    queue <cargo_ants_msgs::Task> tasks_;
    Object * container_;
    Goal pickup_;
    Goal placedown_;
    
    size_t msg_queue_size_;
    string task_topic_;
    ros::Subscriber task_sub_;
    string vehicle_info_topic_;
    ros::Publisher vehicle_info_pub_;
    cargo_ants_msgs::VehicleInfo vehicle_info_;
  };
  
}

#endif // CARGO_ANTS_NPM2_PICK_PLACE_TASK_PROCESS_HPP
