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

#include "ros/ros.h"
#include "cargo_ants_msgs/VehicleInfo.h"
#include "cargo_ants_msgs/ContainerInfo.h"
#include "cargo_ants_msgs/ObstacleMap.h"
#include "cargo_ants_msgs/Task.h"
#include <set>
#include <map>

using namespace std;
using namespace cargo_ants_msgs;

typedef map <string, ContainerInfo> container_info_t;
static container_info_t container_info;

typedef map <string, VehicleInfo> vehicle_info_t;
static vehicle_info_t vehicle_info;

typedef set <string> dispatched_t; // container name
static dispatched_t dispatched;

static ObstacleMap::ConstPtr obstacle_map;


static void container_info_cb (ContainerInfo::ConstPtr const & msg)
{
  container_info[msg->container] = *msg;
}


static void vehicle_info_cb (VehicleInfo::ConstPtr const & msg)
{
  vehicle_info[msg->vehicle] = *msg;
  if (VehicleInfo::PLACING == msg->mode) {
    dispatched.erase (msg->container);
  }
}


static void obstacle_map_cb (ObstacleMap::ConstPtr const & msg)
{
  obstacle_map = msg;
}



int main (int argc, char ** argv)
{
  ros::init (argc, argv, "draft_scheduler");
  ros::NodeHandle node;
  ros::Subscriber container_info_sub (node.subscribe ("/container_info", 100, container_info_cb));
  ros::Subscriber vehicle_info_sub (node.subscribe ("/vehicle_info", 100, vehicle_info_cb));
  ros::Subscriber obstacle_map_sub (node.subscribe ("/obstacle_map", 100, obstacle_map_cb));
  ros::Publisher task_pub (node.advertise<Task> ("/task", 100));
  
  ros::Rate idle_rate(10);
  Task task;
  
  while (ros::ok()) {
    
    // cout << "==================================================\n"
    // 	 << "available vehicles:";
    
    set <string> available_vehicles;
    for (vehicle_info_t::const_iterator iv (vehicle_info.begin());
	 vehicle_info.end() != iv; ++iv) {
      if (VehicleInfo::IDLE == iv->second.mode) {
	available_vehicles.insert (iv->first);
	//	cout << " " << iv->first;
      }
    }
    
    //    cout << "\nwaiting containers:";
    
    set <string> waiting_containers;
    for (container_info_t::const_iterator ic (container_info.begin());
	 container_info.end() != ic; ++ic) {
      if ((ContainerInfo::LIFTED == ic->second.state)
	  && (0 == dispatched.count (ic->first))) {
	waiting_containers.insert (ic->first);
	//	cout << " " << ic->first;
      }
    }
    
    //    cout << "\ndispatched:";
    // for (set <string> ::const_iterator id (dispatched.begin()); dispatched.end() != id; ++id) {
    //   cout << " " << *id;
    // }
    // cout << "\nassignments:";
    
    while ( ! (available_vehicles.empty() || waiting_containers.empty())) {
      task.vehicle = *available_vehicles.begin();
      task.container = *waiting_containers.begin();
      ContainerInfo const & ci (container_info[task.container]);
      task.pickup.gx = ci.pickup_x;
      task.pickup.gy = ci.pickup_y;
      task.pickup.gth = ci.pickup_th;
      task.pickup.dr = 0.1;
      task.pickup.dth = 0.05;
      task.placedown.gx = ci.placedown_x;
      task.placedown.gy = ci.placedown_y;
      task.placedown.gth = ci.placedown_th;
      task.placedown.dr = 0.1;
      task.placedown.dth = 0.05;
      task_pub.publish (task);
      
      dispatched.insert (task.container);
      available_vehicles.erase (task.vehicle);
      waiting_containers.erase (task.container);
      
      //      cout << " " << task.vehicle << "/" << task.container;
    }
    
    //    cout << "\n";
    
    if (!obstacle_map) {
      cout << "we have no map\n";
    }
    
    idle_rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}
