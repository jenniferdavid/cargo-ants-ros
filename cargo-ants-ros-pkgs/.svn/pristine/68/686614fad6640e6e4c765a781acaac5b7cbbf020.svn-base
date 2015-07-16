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
#include "cargo_ants_msgs/VehicleState.h"
#include "cargo_ants_msgs/Path.h"
#include "cargo_ants_msgs/PathStatus.h"
#include "cargo_ants_msgs/Trajectory.h"
#include "cargo_ants_util/util.h"
#include <list>

using namespace cargo_ants_msgs;
using namespace cargo_ants_util;


namespace path_adaptor {
  
  
  class PathAdaptor
  {
  public:
    explicit PathAdaptor (std::string const & name)
      : name_ (name),
	pose_ (Eigen::Vector3d::Zero()),
	active_goal_ (0),
	prev_active_goal_ (0)
    {
      ros::NodeHandle node;
      vehicle_state_sub_ = node.subscribe (name + "/vehicle_state", 10, &PathAdaptor::vehicleStateCB, this);
      path_sub_ = node.subscribe (name + "/path", 10, &PathAdaptor::pathCB, this);
      trajectory_pub_ = node.advertise<Trajectory> (name + "/trajectory", 10);
      path_status_pub_ = node.advertise<PathStatus> (name + "/path_status", 10);
    }
    
    
    void vehicleStateCB (VehicleState::ConstPtr const & msg)
    {
      pose_ = pose3Dto2D (msg->location, msg->orientation);
    }
    
    
    void pathCB (Path::ConstPtr const & msg)
    {
      path_queue_.push_back (msg);
    }
    
    
    void update ()
    {
      if ( ! path_queue_.empty()) {
	
	PathStatus status;
	
	if ( ! goals_.empty()) {
	  status.path_id = path_id_;
	  status.status = PathStatus::PREEMPTED;
	  path_status_pub_.publish (status);
	  ROS_INFO ("path %llu preempted by %s", path_id_, name_.c_str());
	}
	goals_.clear();
	
	for (std::vector<Goal>::const_iterator ip (path_queue_.back()->goals.begin()); ip != path_queue_.back()->goals.end(); ++ip) {
	  goals_.push_back (*ip);
	}
	prev_active_goal_ = -1;
	active_goal_ = 0;
	path_id_ = path_queue_.back()->path_id;
	path_queue_.pop_back();
	
	for (path_queue_t::iterator ip (path_queue_.begin()); ip != path_queue_.end(); ++ip) {
	  status.path_id = (*ip)->path_id;
	  status.status = PathStatus::SKIPPED;
	  path_status_pub_.publish (status);
	  ROS_INFO ("path %llu skipped by %s", (*ip)->path_id, name_.c_str());
	}
	
	ROS_INFO ("path %llu accepted by %s", path_id_, name_.c_str());
	
	path_queue_.clear();
      }
      
      while (active_goal_ < goals_.size()) {
	
	// Depending on how we deploy this, the current velocity also
	// matters for finding out whether the goal has been reached.
	// In particular, if we are moving at a speed which would not
	// allow to come to standstill within the goal tolerance, then
	// the goal is not reached.
	
	if (goalReached (goals_[active_goal_], pose_, true)) {
	  ROS_INFO ("goal %zu reached by %s", active_goal_, name_.c_str());
	  ++active_goal_;
	}
	else {
	  break;
	}
      }
      
      if (active_goal_ >= goals_.size()) {

	// Potentially we could be in situations where a new path has
	// been requested, we're already at the final goal of that
	// path, but there still is a trajectory from a previous path
	// request running. In that case, we should send an empty
	// trajectory to make the vehicle stop. However, the adopted
	// solution may well end up using actionlib, in which case it
	// will be much simpler to cancel the running trajectory.

	if (prev_active_goal_ != active_goal_) {
	  PathStatus status;
	  status.path_id = path_id_;
	  status.status = PathStatus::DONE;
	  path_status_pub_.publish (status);
	}
	// else { we've already handled this }
	
      }
      else {
	
	// to do: implement the real thing... for now just put the
	// current and the goal pose. Also think about whether and how
	// to communicate the angle tolerance.
	
	if (prev_active_goal_ != active_goal_) {
	  PathStatus status;
	  status.path_id = path_id_;
	  status.status = PathStatus::ACTIVE;
	  status.active_goal = active_goal_;
	  path_status_pub_.publish (status);
	}
	
	Trajectory trajectory;
	TrajectoryPoint point;
	point.x = pose_[0];
	point.y = pose_[1];
	point.th = pose_[2];
	trajectory.points.push_back (point);
	point.x = goals_[active_goal_].gx;
	point.y = goals_[active_goal_].gy;
	point.th = goals_[active_goal_].gth;
	trajectory.points.push_back (point);
	trajectory_pub_.publish (trajectory);
	
      }
      
      prev_active_goal_ = active_goal_;
    }
    
  private:
    std::string name_;
    Eigen::Vector3d pose_;
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher path_status_pub_;
    typedef std::list <Path::ConstPtr> path_queue_t;
    path_queue_t path_queue_;
    std::vector <Goal> goals_;
    size_t active_goal_;
    size_t prev_active_goal_;
    uint64_t path_id_;

  };
  
}

using namespace path_adaptor;


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "path_adaptor");
  
  if (argc < 2) {
    ROS_FATAL ("Please specify the vehicle names on the command line");
    return 1;
  }
  
  typedef std::list <boost::shared_ptr <PathAdaptor> > path_adaptors_t;
  path_adaptors_t path_adaptors;
  for (int ii (1); ii < argc; ++ii) {
    path_adaptors.push_back (boost::shared_ptr <PathAdaptor> (new PathAdaptor (argv[ii])));
  }
  
  ros::Rate idle_rate (10);
  while (ros::ok()) {
    for (path_adaptors_t::iterator ip (path_adaptors.begin()); ip != path_adaptors.end(); ++ip) {
      (*ip)->update();
    }
    idle_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
