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

#include "ContainerManager.hpp"
#include "PickPlaceTaskProcess.hpp"
#include "PickPlacePathProcess.hpp"

#include <npm2/Plugin.hpp>
#include <npm2/Object.hpp>
#include <npm2/Simulator.hpp>
#include <npm2/Factory.hpp>
#include <sfl/util/Frame.hpp>
#include <limits>
#include <cmath>
#include <set>
#include <stdlib.h>
#include <sys/time.h>

#include "ros/ros.h"
#include "cargo_ants_msgs/ContainerInfo.h"
#include "cargo_ants_msgs/ObstacleMap.h"

using namespace npm2;
using namespace cargo_ants_npm2;


struct ContainerInfo {
  explicit ContainerInfo (Object * container)
    : container_ (container)
  {
    msg_.state = cargo_ants_msgs::ContainerInfo::OFFSITE;
    msg_.container = container->name;
  }
  
  Object * container_;
  cargo_ants_msgs::ContainerInfo msg_;
  size_t pickup_idx_;
  size_t placedown_idx_;
  double timer_;
};


class ContainerTeleport
  : public SimulatorHook,
    public ContainerManager
{
public:
  explicit ContainerTeleport (string const & name);
  
  // SimulatorHook
  //
  virtual bool init (ostream & err);
  virtual void preActuation (ostream & err);
  virtual void preSensing (ostream & err);
  virtual void preProcessing (ostream & err);
  
  // ContainerManager
  //
  virtual bool grabContainer (string const & vehicle, string const & container);
  virtual bool releaseContainer (string const & vehicle, string const & container);
  
  bool randomPickPlace (ContainerInfo & info);
  
  double wait_;
  vector <Object *> containers_;
  typedef map <string, ContainerInfo> container_info_t;
  container_info_t container_info_;
  vector <sfl::Frame> pickups_;
  set <size_t> taken_pickups_;
  vector <sfl::Frame> placedowns_;
  set <size_t> taken_placedowns_;
  
  string container_info_topic_;
  ros::Publisher container_info_pub_;
  
  string obstacle_map_topic_;
  ros::Publisher obstacle_map_pub_;
};


//////////////////////////////////////////////////


int npm2_plugin_init ()
{
  if ( ! ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "cargo_ants_npm2",
	      ros::init_options::NoSigintHandler |
	      ros::init_options::AnonymousName);
  }
  
  Factory::instance().declare <ContainerTeleport> ("ContainerTeleport");
  Factory::instance().declare <PickPlaceTaskProcess> ("PickPlaceTaskProcess");
  Factory::instance().declare <PickPlacePathProcess> ("PickPlacePathProcess");
  
  struct timeval tt;
  gettimeofday (&tt, NULL);
  srand (tt.tv_usec);
  
  return 0;
}


//////////////////////////////////////////////////


ContainerTeleport::
ContainerTeleport (string const & name)
  : ContainerManager (name),
    wait_ (1.0),
    container_info_topic_ ("container_info"),
    obstacle_map_topic_ ("obstacle_map")
{
  reflectParameter ("wait", &wait_);
  reflectVectorSlot ("containers", &containers_);
  reflectVectorParameter ("pickups", &pickups_);
  reflectVectorParameter ("placedowns", &placedowns_);
  reflectParameter ("container_info_topic", &container_info_topic_);
  reflectParameter ("obstacle_map_topic", &obstacle_map_topic_);
  
  Simulator::instance()->addHook (true, this);
}


bool ContainerTeleport::
init (ostream & err)
{
  if (containers_.empty()) {
    err << "ContainerTeleport: no containers\n";
    return false;
  }
  if (pickups_.empty()) {
    err << "ContainerTeleport: no pickups\n";
    return false;
  }
  if (placedowns_.empty()) {
    err << "ContainerTeleport: no placedowns\n";
    return false;
  }

  ros::NodeHandle node;
  container_info_pub_ = node.advertise <cargo_ants_msgs::ContainerInfo> (container_info_topic_,
									 2 * containers_.size());
  obstacle_map_pub_ = node.advertise <cargo_ants_msgs::ObstacleMap> (obstacle_map_topic_, 10);
  
  container_info_.clear();
  for (size_t ii (0); ii < containers_.size(); ++ii) {
    container_info_.insert (make_pair (containers_[ii]->name, ContainerInfo (containers_[ii])));
    containers_[ii]->attach (0);
  }
  taken_pickups_.clear();
  taken_placedowns_.clear();
  
  return true;
}


void ContainerTeleport::
preActuation (ostream & err)
{
  for (container_info_t::iterator ic (container_info_.begin());
       container_info_.end() != ic; ++ic) {
    
    switch (ic->second.msg_.state) {
    
    case cargo_ants_msgs::ContainerInfo::OFFSITE:
      if (randomPickPlace (ic->second)) {
	ic->second.msg_.state = cargo_ants_msgs::ContainerInfo::LIFTED;
	ic->second.msg_.pickup_x     = pickups_   [ic->second.pickup_idx_]   .X();
	ic->second.msg_.pickup_y     = pickups_   [ic->second.pickup_idx_]   .Y();
	ic->second.msg_.pickup_th    = pickups_   [ic->second.pickup_idx_]   .Theta();
	ic->second.msg_.placedown_x  = placedowns_[ic->second.placedown_idx_].X();
	ic->second.msg_.placedown_y  = placedowns_[ic->second.placedown_idx_].Y();
	ic->second.msg_.placedown_th = placedowns_[ic->second.placedown_idx_].Theta();
	taken_pickups_.insert (ic->second.pickup_idx_);
	taken_placedowns_.insert (ic->second.placedown_idx_);
	ic->second.container_->attach (Simulator::world());
	ic->second.container_->mount_ = pickups_[ic->second.pickup_idx_];
	ic->second.container_->motion_.Set (0.0, 0.0, 0.0);
      }
      break;
      
    case cargo_ants_msgs::ContainerInfo::PLACED:
      ic->second.timer_ += Simulator::instance()->timestep_;
      if (ic->second.timer_ > wait_) {
	size_t const old_pickup_idx (ic->second.pickup_idx_);
	size_t const old_placedown_idx (ic->second.placedown_idx_);
	taken_placedowns_.erase (old_placedown_idx);
	if ( ! randomPickPlace (ic->second)) {
	  taken_placedowns_.insert (old_placedown_idx);
	}
	else {
	  ic->second.msg_.state = cargo_ants_msgs::ContainerInfo::LIFTED;
	  ic->second.msg_.pickup_x     = pickups_   [ic->second.pickup_idx_]   .X();
	  ic->second.msg_.pickup_y     = pickups_   [ic->second.pickup_idx_]   .Y();
	  ic->second.msg_.pickup_th    = pickups_   [ic->second.pickup_idx_]   .Theta();
	  ic->second.msg_.placedown_x  = placedowns_[ic->second.placedown_idx_].X();
	  ic->second.msg_.placedown_y  = placedowns_[ic->second.placedown_idx_].Y();
	  ic->second.msg_.placedown_th = placedowns_[ic->second.placedown_idx_].Theta();
	  taken_pickups_.erase (old_pickup_idx);
	  taken_pickups_.insert (ic->second.pickup_idx_);
	  taken_placedowns_.insert (ic->second.placedown_idx_);
	  ic->second.container_->mount_ = pickups_[ic->second.pickup_idx_];
	  ic->second.container_->motion_.Set (0.0, 0.0, 0.0);
	}
      }
      break;
      
    case cargo_ants_msgs::ContainerInfo::LIFTED:
    case cargo_ants_msgs::ContainerInfo::ATTACHED:
    default:
      break;
      //  Just ignore the other states.  They're handled by
      //  grabContainer and releaseContainer.
    }
    
    container_info_pub_.publish (ic->second.msg_);
  }
  
  npm2::Body::lines_t const & lines (npm2::Simulator::world()->body_.getLines());
  cargo_ants_msgs::Obstacle obstacle;
  obstacle.origin.ox = 0.0;
  obstacle.origin.oy = 0.0;
  obstacle.origin.oth = 0.0;
  for (size_t ii (0); ii < lines.size(); ++ii) {
    cargo_ants_msgs::Point p0, p1;
    p0.px = lines[ii].X0();
    p0.py = lines[ii].Y0();
    p1.px = lines[ii].X1();
    p1.py = lines[ii].Y1();
    cargo_ants_msgs::Polyline polyline;
    polyline.points.push_back (p0);
    polyline.points.push_back (p1);
    obstacle.polylines.push_back (polyline);
  }
  cargo_ants_msgs::ObstacleMap msg;
  msg.obstacles.push_back (obstacle);
  obstacle_map_pub_.publish (msg);
}


bool ContainerTeleport::
randomPickPlace (ContainerInfo & info)
{
  if ((taken_pickups_.size() >= pickups_.size())
      || (taken_placedowns_.size() >= placedowns_.size())) {
    return false;
  }
  
  size_t nfree (pickups_.size() - taken_pickups_.size());
  size_t pickup_idx (((size_t) rand()) % nfree);
  for (size_t ii(1); ii < pickups_.size(); ++ii) {
    if (0 == taken_pickups_.count (pickup_idx)) {
      break;
    }
    ++pickup_idx;
  }
  if (0 != taken_pickups_.count (pickup_idx)) {
    ROS_ERROR ("randomPickPlace: pickup index taken in spite of search");
    return false;
  }
  
  nfree = placedowns_.size() - taken_placedowns_.size();
  size_t placedown_idx (((size_t) rand()) % nfree);
  for (size_t ii(1); ii < placedowns_.size(); ++ii) {
    if (0 == taken_placedowns_.count (placedown_idx)) {
      break;
    }
    ++placedown_idx;
  }
  if (0 != taken_placedowns_.count (placedown_idx)) {
    ROS_ERROR ("randomPickPlace: placedown index taken in spite of search");
    return false;
  }
  
  info.pickup_idx_ = pickup_idx;
  info.placedown_idx_ = placedown_idx;
  return true;
}


void ContainerTeleport::
preSensing (ostream & err)
{
}


void ContainerTeleport::
preProcessing (ostream & err)
{
}


bool ContainerTeleport::
grabContainer (string const & vehicle, string const & container)
{
  // Fail if the container is unkown or neither LIFTED nor PLACED.
  // I.e. it is not currently attached to another vehicle, nor is it
  // OFFSITE.
  //
  // As an alternative, if it is already attached to the given
  // vehicle, maybe it can silently succeed.  But that would actually
  // indicate a possible bug, so it is more prudent to fail.
  //
  // One idea is to fail if the vehicle is currently carrying another
  // container.  But vehicles may have capacity for more than one
  // container.  That should probably be managed separately, so for
  // now siltenly assume we have enough space on the vehicle.
  //
  container_info_t::iterator ic (container_info_.find (container));
  if (container_info_.end() == ic) {
    ROS_ERROR ("grabContainer(): unknown container %s", container.c_str());
    return false;
  }
  if ((cargo_ants_msgs::ContainerInfo::LIFTED != ic->second.msg_.state)
      && (cargo_ants_msgs::ContainerInfo::PLACED != ic->second.msg_.state)) {
    ROS_ERROR ("grabContainer(): container %s has state %d (neither LIFTED %d nor PLACED %d)",
	       container.c_str(), ic->second.msg_.state,
	       cargo_ants_msgs::ContainerInfo::LIFTED, cargo_ants_msgs::ContainerInfo::PLACED);
    return false;
  }
  
  // In one sense, it is silly to use name-based lookup here (the
  // caller is likely to already have pointers to the Objects).  But
  // this will scale better when we distribute stuff and it runs
  // outside the simulator.
  //
  Object * vv (Object::registry.find (vehicle));
  if ( ! vv) {
    ROS_ERROR ("grabContainer(): vehicle %s not found", vehicle.c_str());
    return false;
  }
  Object * cc (Object::registry.find (container));
  if ( ! cc) {
    ROS_ERROR ("grabContainer(): container %s not found", container.c_str());
    return false;
  }
  
  if (cargo_ants_msgs::ContainerInfo::LIFTED == ic->second.msg_.state) {
    taken_pickups_.erase (ic->second.pickup_idx_);
  }
  else if (cargo_ants_msgs::ContainerInfo::PLACED == ic->second.msg_.state) {
    taken_placedowns_.erase (ic->second.placedown_idx_);
  }
  ic->second.msg_.state = cargo_ants_msgs::ContainerInfo::ATTACHED;
  ic->second.msg_.vehicle = vehicle;
  cc->attach (vv);
  
  return true;
}


bool ContainerTeleport::
releaseContainer (string const & vehicle, string const & container)
{
  // Fail if the container is not known or is not actually ATTACHED to
  // the given vehicle.  Alternatively, it may be OK to suceed if the
  // container is not attached to anything at the moment... but a
  // mismatched request probably indicates a bug somewhere.
  //
  container_info_t::iterator ic (container_info_.find (container));
  if (container_info_.end() == ic) {
    ROS_ERROR ("releaseContainer(): unknown container %s", container.c_str());
    return false;
  }
  if (cargo_ants_msgs::ContainerInfo::ATTACHED != ic->second.msg_.state) {
    ROS_ERROR ("releaseContainer(): container %s is not ATTACHED", container.c_str());
    return false;
  }
  if (vehicle != ic->second.msg_.vehicle) {
    ROS_ERROR ("releaseContainer(): container %s is attached to %s instead of %s",
	       container.c_str(), ic->second.msg_.vehicle.c_str(), vehicle.c_str());
    return false;
  }
  
  Object * cc (Object::registry.find (container));
  if ( ! cc) {
    ROS_ERROR ("releaseContainer(): container %s not found", container.c_str());
    return false;
  }
  
  // Now here's a problem.  How do we know or verify at this stage
  // which placedown index to block?  There is some unclean crosstalk
  // between the randomized teleportation business and this supposedly
  // clean container manager.  For now, just assume that it has indeed
  // been placed where its placedown_idx stipulates.
  //
  taken_placedowns_.insert (ic->second.placedown_idx_);
  
  ic->second.msg_.state = cargo_ants_msgs::ContainerInfo::PLACED;
  ic->second.msg_.vehicle = "";
  ic->second.timer_ = 0.0;
  cc->attach (Simulator::world());
  
  return true;
}
