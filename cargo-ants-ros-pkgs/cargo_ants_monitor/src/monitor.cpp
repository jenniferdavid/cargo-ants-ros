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


using namespace cargo_ants_msgs;
using namespace cargo_ants_util;


namespace {
  
  
  template <typename MessageType>
  struct MessageInfo {
    typedef MessageType msg_t;
    
    uint64_t id;
    ros::Time time;
    typename MessageType::ConstPtr msg;
    
    MessageInfo () {}
    
    MessageInfo (uint64_t _id, typename MessageType::ConstPtr _msg)
      : id (_id), time (ros::Time::now()), msg (_msg) {}
  };
  
  
  template <typename MessageType>
  uint64_t qhGetID (typename MessageType::ConstPtr const & msg);
  
  template <>
  uint64_t qhGetID <cargo_ants_msgs::Path> (cargo_ants_msgs::Path::ConstPtr const & msg)
  { return msg->path_id; }
  
  template <>
  uint64_t qhGetID <cargo_ants_msgs::PathStatus> (cargo_ants_msgs::PathStatus::ConstPtr const & msg)
  { return msg->path_id; }
  
  template <>
  uint64_t qhGetID <cargo_ants_msgs::Trajectory> (cargo_ants_msgs::Trajectory::ConstPtr const & msg)
  { return 0; }
  
  
  template <typename MessageType>
  struct TopicInfo {
    typedef TopicInfo <MessageType> MyType;
    
    std::string name;
    ros::Subscriber sub;
    typedef typename std::map <uint64_t, MessageInfo <MessageType> > msg_info_t;
    msg_info_t msg_info;
    
    void init (std::string const & name, uint64_t queue_size)
    {
      this->name = name;
      ros::NodeHandle node;
      sub = node.subscribe (name, queue_size, &MyType::cb, this);
    }
    
    void cb (typename MessageType::ConstPtr const & msg)
    {
      uint64_t const id (qhGetID <MessageType> (msg));
      msg_info[id] = MessageInfo <MessageType> (id, msg);
    }
  };
  
  
  class VehicleMonitor
  {
  public:
    VehicleMonitor (std::string const & name, double ttl_done_sec)
      : name_ (name),
	ttl_done_ (ttl_done_sec),
	pose_ (Eigen::Vector3d::Zero())
    {
      std::cout << "creating VehicleMonitor for " << name << "\n";
      
      ros::NodeHandle node;
      vehicle_state_sub_ = node.subscribe (name + "/vehicle_state", 10,
					   &VehicleMonitor::vehicleStateCB, this);
      path_.init (name + "/path", 10);
      path_status_.init (name + "/path_status", 10);
      trajectory_.init (name + "/trajectory", 10);
    }
    
    void vehicleStateCB (VehicleState::ConstPtr const & msg)
    {
      pose_ = pose3Dto2D (msg->location, msg->orientation);
    }
    
    void update ()
    {
      ros::Time const now (ros::Time::now());
      
      std::cout << "--------------------------------------------------\n"
		<< "Vehicle " << name_ << "\n";
      
      prettyPrint (pose_, std::cout, "  pose", "    ");
      
      std::cout << "Path: " << path_.name << "\n";
      for (path_t::msg_info_t::const_iterator ip (path_.msg_info.begin()); ip != path_.msg_info.end(); ++ip) {
	MessageInfo <Path> const & info (ip->second);
	std::cout << "  ID: " << info.id << "   age: " << now - info.time << "\n";
	std::vector <Goal> const & goals (info.msg->goals);
	for (size_t ig (0); ig < goals.size(); ++ig) {
	  std::cout << "  goal [" << ig << "]:   ";
	  prettyPrint (goals[ig].gx, std::cout);
	  prettyPrint (goals[ig].gy, std::cout);
	  prettyPrint (goals[ig].gth, std::cout);
	  std::cout << "   ";
	  prettyPrint (goals[ig].dr, std::cout);
	  prettyPrint (goals[ig].dth, std::cout);
	  std::cout << "\n";
	}
      } 
      
      std::cout << "PathStatus: " << path_status_.name << "\n";
      for (path_status_t::msg_info_t::const_iterator ips (path_status_.msg_info.begin()); ips != path_status_.msg_info.end(); ++ips) {
	MessageInfo <PathStatus> const & info (ips->second);
	std::cout << "  ID: " << info.id << "   age: " << now - info.time << "\n";
	switch (info.msg->status) {
	case PathStatus::DONE:
	  std::cout << "  status: DONE\n";
	  break;
	case PathStatus::ACTIVE:
	  std::cout << "  status: ACTIVE at goal [" << info.msg->active_goal << "]\n";
	  break;
	case PathStatus::SKIPPED:
	  std::cout << "  status: SKIPPED\n";
	  break;
	case PathStatus::PREEMPTED:
	  std::cout << "  status: PREEMPTED\n";
	  break;
	default:
	  std::cout << "  status: " << info.msg->status << "\n";
	  break;
	}
      }
      
      std::cout << "Trajectory: " << trajectory_.name << "\n";
      for (trajectory_t::msg_info_t::const_iterator it (trajectory_.msg_info.begin()); it != trajectory_.msg_info.end(); ++it) {
	MessageInfo <Trajectory> const & info (it->second);
	std::cout << "  ID: " << info.id << "   age: " << now - info.time << "\n";
	std::vector <TrajectoryPoint> const & points (info.msg->points);
	for (size_t ip (0); ip < points.size(); ++ip) {
	  std::cout << "  point [" << ip << "]:   ";
	  prettyPrint (points[ip].x, std::cout);
	  prettyPrint (points[ip].y, std::cout);
	  prettyPrint (points[ip].th, std::cout);
	  std::cout << "\n";
	  if ((points.size() > 6) && (ip == 2)) {
	    std::cout << "    ...\n";
	    ip = points.size() - 4;
	  }
	}
      }
      
    }
    
  private:
    std::string name_;
    ros::Duration ttl_done_;
    
    ros::Subscriber vehicle_state_sub_;
    Eigen::Vector3d pose_;
    
    typedef TopicInfo <Path> path_t;
    path_t path_;
    
    typedef TopicInfo <PathStatus> path_status_t;
    path_status_t path_status_;
    
    typedef TopicInfo <Trajectory> trajectory_t;
    trajectory_t trajectory_;
  };
  
}


int main (int argc, char ** argv)
{
  ros::init (argc, argv, "cargo_ants_monitor");
  
  if (argc < 2) {
    ROS_FATAL ("Please specify the vehicle names on the monitor command line");
    return 1;
  }
  
  typedef std::list <boost::shared_ptr <VehicleMonitor> > monitors_t;
  monitors_t monitors;
  for (int ii (1); ii < argc; ++ii) {
    monitors.push_back (boost::shared_ptr <VehicleMonitor> (new VehicleMonitor (argv[ii], 5.0)));
  }
  
  ros::Rate rate (1);
  while (ros::ok()) {
    std::cout << "\n**************************************************\n";
    for (monitors_t::iterator im (monitors.begin()); im != monitors.end(); ++im) {
      (*im)->update();
    }
    rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}
