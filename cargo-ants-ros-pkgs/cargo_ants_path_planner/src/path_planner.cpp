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
#include "cargo_ants_msgs/Task.h"
#include "cargo_ants_msgs/Path.h"
#include "cargo_ants_msgs/ObstacleMap.h"
#include "cargo_ants_msgs/VehicleInfo.h"
#include "cargo_ants_util/util.h"
#include "cargo_ants_util/estar_util.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <estar2/estar.h>
#include <boost/scoped_ptr.hpp>
#include <list>

using namespace cargo_ants_msgs;
using namespace cargo_ants_util;


namespace path_planner {
  
  
  class PathPlanner
  {
  public:
    explicit PathPlanner (std::string const & name);
    
    void vehicleInfoCB (VehicleInfo::ConstPtr const & msg);
    void taskCB (Task::ConstPtr const & msg);
    void siteMapCB (ObstacleMap::ConstPtr const & msg);
    
    void update ();

  protected:
    bool createPath (Goal const & task_goal,
		     std::vector<Goal> & path_goals);
    
    std::string name_;
    
    ros::Subscriber vehicle_info_sub_;
    VehicleInfo::ConstPtr vehicle_info_;
    
    ros::Subscriber task_sub_;
    typedef std::list <Task::ConstPtr> task_queue_t;
    task_queue_t task_queue_;
    Task::ConstPtr task_;
    
    ros::Subscriber site_map_sub_;
    ObstacleMap::ConstPtr site_map_;
    
    ros::Publisher path_pub_;
    Path path_;
    
    double map_resolution_;
    double robot_radius_;
    double buffer_zone_;
    double decay_power_;
    double via_goal_dr_;
    double via_goal_dth_;
    double final_goal_dr_;
    double final_goal_dth_;
  };
  
}


//////////////////////////////////////////////////

using namespace path_planner;


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "path_planner");
  
  if (argc < 2) {
    ROS_FATAL ("Please specify the vehicle names on the command line");
    return 1;
  }
  
  typedef std::list <boost::shared_ptr <PathPlanner> > path_planners_t;
  path_planners_t path_planners;
  for (int ii (1); ii < argc; ++ii) {
    path_planners.push_back (boost::shared_ptr <PathPlanner> (new PathPlanner (argv[ii])));
  }
  
  ros::Rate idle_rate (10);
  while (ros::ok()) {
    for (path_planners_t::iterator ip (path_planners.begin()); ip != path_planners.end(); ++ip) {
      (*ip)->update();
    }
    idle_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}


//////////////////////////////////////////////////

namespace path_planner {
  
  
  PathPlanner::
  PathPlanner (std::string const & name)
    : name_ (name),
      map_resolution_ (0.5),
      robot_radius_ (0.5),
      buffer_zone_ (1.5),
      decay_power_ (3.0),
      via_goal_dr_ (1.0),
      via_goal_dth_ (M_PI),
      final_goal_dr_ (0.3),
      final_goal_dth_ (0.05)
  {
    ros::NodeHandle node;
    
    vehicle_info_sub_ = node.subscribe ("/vehicle_info", 10, // bcast, later probably local
					&PathPlanner::vehicleInfoCB, this);
    site_map_sub_ = node.subscribe ("/site_map", // bcast
				    10, &PathPlanner::siteMapCB, this);
    
    task_sub_ = node.subscribe ("/task", // bcast
				10, &PathPlanner::taskCB, this);
    
    path_pub_ = node.advertise<Path> (name + "/path", // local
				      10);
    
    path_.mode = Path::IDLE;
  }
  
  
  void PathPlanner::
  vehicleInfoCB (VehicleInfo::ConstPtr const & msg)
  {
    if (msg->vehicle == name_) {
      vehicle_info_ = msg;
    }
  }
  
  
  void PathPlanner::
  taskCB (Task::ConstPtr const & msg)
  {
    if (msg->vehicle == name_) {
      task_queue_.push_back (msg);
    }
  }
  
  
  void PathPlanner::
  siteMapCB (ObstacleMap::ConstPtr const & msg)
  {
    site_map_ = msg;
  }
  
  
  void PathPlanner::
  update ()
  {
    if ( ! task_queue_.empty()) {
      
      // At some point we might want to check whether we're being
      // asked to pick up a container while still caryying another
      // one.  In that case, either return an error status, or
      // implement a proper queuing system.  For now, just assume that
      // never happens.
	
      path_.goals.clear();
      if (Path::IDLE != path_.mode) {
	// We got interrupted.
	path_.mode = Path::ABORT;
	path_pub_.publish (path_);
      }
	
      // If there happen to be more than one task in the queue, just
      // ignore everything except the last (most recent) one.
	
      task_ = task_queue_.back();
      task_queue_.clear();
	
      path_.mode = Path::PICKUP;
      path_.container = task_->container;
      if ( ! createPath (task_->pickup, path_.goals)) {
	ROS_ERROR ("The unthinkable happened! lkjhgfds");
      }
      else {
	path_pub_.publish (path_);
      }
	
      return;
    }
      
    // When we didn't get a new task, just monitor the vehicle info to
    // detect when it needs the path from pickup to placedown
    // location.
      
    switch (path_.mode) {
	
      // --------------------------------------------------
    case Path::PICKUP:
      if (vehicle_info_->mode == VehicleInfo::PLACING) {
	path_.mode = Path::PLACEDOWN;
	path_.goals.clear();
	if ( ! createPath (task_->placedown, path_.goals)) {
	  ROS_ERROR ("The unthinkable happened! lknefwyuv678");
	}
	else {
	  path_pub_.publish (path_);
	}
      }
      break;
	
      // --------------------------------------------------
    case Path::PLACEDOWN:
      if (vehicle_info_->mode == VehicleInfo::IDLE) {
	path_.mode = Path::IDLE;
	path_.goals.clear();
	path_pub_.publish (path_);
      }
      break;
	
      // --------------------------------------------------
    case Path::ABORT:
    case Path::IDLE:
    default:
      // do nothing
      break;
    }
  }
  
  
  bool PathPlanner::
  createPath (Goal const & task_goal,
	      std::vector<Goal> & path_goals)
  {
    if ( ! vehicle_info_) {
      ROS_ERROR ("%s cannot create path: no vehicle info yet", name_.c_str());
      return false;
    }
      
    if ( ! site_map_) {
      ROS_ERROR ("%s cannot create path: no site map yet", name_.c_str());
      return false;
    }
    
    ROS_INFO ("%s creating path (%g   %g   %g) to (%g   %g   %g)",
	      name_.c_str(),
	      vehicle_info_->vehicle_px, vehicle_info_->vehicle_py, vehicle_info_->vehicle_pth,
	      task_goal.gx, task_goal.gy, task_goal.gth);
    
    EstarHelper eh;
    // XXXX could add a little buffer all around the costmap...
    eh.readMap (site_map_,
		std::min (vehicle_info_->vehicle_px, task_goal.gx),
		std::min (vehicle_info_->vehicle_py, task_goal.gy),
		std::max (vehicle_info_->vehicle_px, task_goal.gx),
		std::max (vehicle_info_->vehicle_py, task_goal.gy),
		map_resolution_, robot_radius_, buffer_zone_, decay_power_);
    int errcode;
    boost::scoped_ptr <DistanceTransform>
      dt (eh.createDistanceTransform (task_goal.gx, task_goal.gy, &errcode));
    if ( ! dt) {
      ROS_ERROR ("%s distance transform error code %d", name_.c_str(), errcode);
      return false;
    }
      
    DistanceTransform::trace_t trace;
    if ( ! dt->trace (vehicle_info_->vehicle_px, vehicle_info_->vehicle_py,
		      0.5 * via_goal_dr_, trace, &errcode)) {
      ROS_ERROR ("%s trace error code %d", name_.c_str(), errcode);
      return false;
    }
    if (trace.size() < 2) {
      ROS_ERROR ("%s trace contains only %zu points", name_.c_str(), trace.size());
      return false;
    }
      
    path_goals.clear();
    Goal gg;
    gg.dr = via_goal_dr_;
    gg.dth = via_goal_dth_;
    for (size_t ii (0); ii < trace.size() - 1; ++ii) {
      gg.gx = trace[ii].posx;
      gg.gy = trace[ii].posy;
      gg.gth = atan2 (trace[ii].grady, trace[ii].gradx);
      path_goals.push_back (gg);
    }
    gg.gx = task_goal.gx;
    gg.gy = task_goal.gy;
    gg.gth = task_goal.gth;
    gg.dr = final_goal_dr_;
    gg.dth = final_goal_dth_;
    path_goals.push_back (gg);
      
    return true;
  }

}

