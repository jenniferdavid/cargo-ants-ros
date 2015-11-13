/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#ifndef NPM_ASLBOT_HPP
#define NPM_ASLBOT_HPP


#include <npm/common/RobotClient.hpp>
#include <sfl/util/vec2d.hpp>
#include <vector>


namespace asl {
  struct path_element;
  typedef std::vector<path_element> path_t;
	typedef sfl::vec2d<double> path_point;
	typedef std::vector<path_point> trajectory_t;
  class Algorithm;
	class MappingThread;
	class PlanningThread;
  class ControlThread;
	struct travmap_grow_options;
} 


namespace sfl {
  class Multiscanner;
  class Odometry;
	class RWlock;
}


namespace estar {
	class AlgorithmOptions;
}

class AslColorScheme;
struct smartparams;


class AslBot
  : public npm::RobotClient
{
private:
  AslBot(AslBot const &);
  
public:
  AslBot(boost::shared_ptr<npm::RobotDescriptor> descriptor,
				 npm::World const & world);
	~AslBot();
	
	virtual bool PrepareAction(double timestep);
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
	sfl::Frame const & GetPose() const;
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  virtual bool GoalReached();
	
	virtual const asl::trajectory_t * GetTrajectory() const = 0;
	virtual bool GetRefpoint(asl::path_point &ref_point) const = 0;
	
	/** \note Can return null. */
	void CopyPaths(boost::shared_ptr<asl::path_t> & clean,
								 boost::shared_ptr<asl::path_t> & dirty) const;
	
protected:
  boost::shared_ptr<sfl::Multiscanner> m_mscan;
  boost::shared_ptr<AslColorScheme> m_asl_cs;
	boost::shared_ptr<sfl::Odometry> m_odo;
  boost::shared_ptr<asl::Algorithm> m_asl_algo;
  boost::shared_ptr<sfl::RWlock> m_simul_rwlock;
  boost::shared_ptr<asl::MappingThread> m_mapping_thread;
  boost::shared_ptr<asl::PlanningThread> m_planning_thread;
  boost::shared_ptr<asl::ControlThread> m_control_thread;
	bool m_error;
	int m_simul_usecsleep;
	int m_mapping_usecsleep;
	int m_planning_usecsleep;
	int m_control_usecsleep;
	
	/** \todo Hmmm... a bit of a hack, but not tooooo bad. */
	void CreateMePlease(boost::shared_ptr<npm::RobotDescriptor> descriptor,
											npm::World const & world);
	
	virtual
	void InitAlgorithm(boost::shared_ptr<npm::RobotDescriptor> descriptor,
										 smartparams const & params,
										 double carrot_distance,
										 double carrot_stepsize,
										 size_t carrot_maxnsteps,
										 double replan_distance,
										 std::string const & traversability_file,
										 int estar_step,
										 double wavefront_buffer,
										 std::string const & goalmgr_filename,
										 bool swiped_map_update,
										 double max_swipe_distance,
										 boost::shared_ptr<estar::AlgorithmOptions> estar_options,
										 boost::shared_ptr<asl::travmap_grow_options> grow_options,
										 bool estar_grow_grid,
										 /** \todo XXX aarghhhhh! */
										 double & robot_radius) = 0;
	
	virtual void InitScanners(boost::shared_ptr<sfl::Multiscanner> mscan,
														smartparams const & params) = 0;
	
	virtual void InitDrive(smartparams const & params) = 0;

	virtual void InitBody(smartparams const & params) = 0;
	
	virtual void MoreGraphics(std::string const & name,
														npm::World const & world,
														bool slow_drawing_enabled) = 0;
};

#endif // NPM_ASLBOT_HPP
