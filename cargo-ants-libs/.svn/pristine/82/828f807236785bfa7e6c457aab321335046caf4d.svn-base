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

#include "Borox.hpp"
#include "smart_params.hpp"
#include <asl/Algorithm.hpp>
#include <asl/Planner.hpp>
#include <asl/DWAPathTracker.hpp>
#include <asl/Controller.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/util/Hull.hpp>
#include <sfl/util/GoalManager.hpp>
#include <sfl/util/strutil.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include "../robox/DWDrawing.hpp"
#include "../robox/ODrawing.hpp"
#include "../robox/DODrawing.hpp"
#include "../robox/OCamera.hpp"
#include "../common/Lidar.hpp"
#include "../common/RobotDescriptor.hpp"
#include "../common/pdebug.hpp"
#include "../common/OdometryDrawing.hpp"
#include "../common/StillCamera.hpp"
#include "../common/Manager.hpp"
#include <iostream>


using namespace npm;
using namespace sfl;
using namespace asl;
using namespace boost;
using namespace std;


Borox::
Borox(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : AslBot(descriptor, world)
{
  CreateMePlease(descriptor, world); // calls various hooks...
}


static shared_ptr<Hull> CreateHull()
{
  shared_ptr<Hull> hull(new Hull());
  static const double octoSmall = 0.178;
  static const double octoBig = 0.430;
  
  Polygon outline;		// temporary
  outline.AddPoint( octoBig  , octoSmall);
  outline.AddPoint( octoSmall, octoBig);
  outline.AddPoint(-octoSmall, octoBig);
  outline.AddPoint(-octoBig  , octoSmall);
  outline.AddPoint(-octoBig  ,-octoSmall);
  outline.AddPoint(-octoSmall,-octoBig);
  outline.AddPoint( octoSmall,-octoBig);
  outline.AddPoint( octoBig  ,-octoSmall);
  hull->AddPolygon(outline);
  
  return hull;
}


static shared_ptr<RobotModel> CreateModel(smartparams const & params)
{
  RobotModel::Parameters modelparams(params.model_security_distance,
				     params.model_wheelbase,
				     params.model_wheelradius,
				     params.model_qd_max,
				     params.model_qdd_max,
				     params.model_sd_max,
				     params.model_thetad_max,
				     params.model_sdd_max,
				     params.model_thetadd_max);
  shared_ptr<RobotModel> model(new RobotModel(modelparams, CreateHull()));
  return model;
}


void Borox::
InitAlgorithm(boost::shared_ptr<npm::RobotDescriptor> descriptor,
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
	      double & robot_radius)
{
  if ( ! m_model)
    m_model = CreateModel(params);
  
  robot_radius = m_model->GetSafetyHull()->CalculateRadius();
  
  double buffer_zone(robot_radius);
  string_to(descriptor->GetOption("buffer_zone"), buffer_zone);
  
  bool use_simple_query(false);
  string_to(descriptor->GetOption("use_simple_query"), use_simple_query);
  
#warning 'make these two configurable one day'
  double const padding_factor(2);	// 2 is legacy
  double const decay_power(2); // this is not, however
  
  shared_ptr<Mapper2d::always_grow> grow_strategy(new Mapper2d::always_grow());
  shared_ptr<Mapper2d>
    m2d(Mapper2d::Create(robot_radius, buffer_zone, padding_factor,
			 ////linear_travmap_cost_decay(),
			 shared_ptr<exponential_travmap_cost_decay>(new exponential_travmap_cost_decay(decay_power)),
			 traversability_file, grow_strategy, &cerr));
  if ( ! m2d) {
    cerr << "ERROR in Borox::InitAlgorithm():\n"
	 << "  Could not create Mapper2d instance.\n";
    exit(EXIT_FAILURE);
  }
  shared_ptr<RDTravmap> rdtravmap(m2d->CreateRDTravmap());
  
  shared_ptr<RWlock> plan_rwlock(RWlock::Create("Borox::plan_rwlock"));
  if ( ! plan_rwlock) {
    cerr << "ERROR in Borox::InitAlgorithm():\n"
	 << "  Could not create plan_rwlock.\n";
    exit(EXIT_FAILURE);
  }
  
  shared_ptr<GoalManager> gmgr;
  if (goalmgr_filename != "") {
    gmgr.reset(new GoalManager());
    if ( ! gmgr->ParseConfig(goalmgr_filename, &cerr)) {
      cerr << "ERROR in Borox::InitAlgorithm():\n"
	   << "  GoalManager::ParseConfig() failed\n";
      exit(EXIT_FAILURE);
    }
  }
  
  if ( ! m_dwa)
    m_dwa.reset(new LegacyDynamicWindow(params.dwa_dimension,
					params.dwa_grid_width,
					params.dwa_grid_height,
					params.dwa_grid_resolution,
					m_model,
					params.dwa_alpha_distance,
					params.dwa_alpha_heading,
					params.dwa_alpha_speed,
					true));
  
  if ( ! m_mscan) {
    cerr << "ERROR in Borox::InitAlgorithm():\n"
	 << "  m_mscan is NULL\n";
    exit(EXIT_FAILURE);
  }
  m_dwa_ptrack.reset(new DWAPathTracker(m_dwa, m_dwa->GetSpeedObjective(), m_model, m_mscan));
  
  shared_ptr<Planner> planner;
  if (use_simple_query)
    planner.reset(new FakePlanner(rdtravmap, gmgr, m_dwa_ptrack,
				  carrot_distance, carrot_stepsize,
				  plan_rwlock));
  else if ( ! estar_grow_grid)
    planner.reset(new EstarPlanner(rdtravmap, gmgr, m_dwa_ptrack,
				   estar_options, replan_distance,
				   wavefront_buffer, carrot_distance,
				   carrot_stepsize, carrot_maxnsteps,
				   estar_step, plan_rwlock));
  else {
    size_t coarse_grid_ncells(10);
    string_to(descriptor->GetOption("coarse_grid_ncells"), coarse_grid_ncells);
    planner.reset(new GrowingEstarPlanner(rdtravmap, gmgr, m_dwa_ptrack,
					  estar_options, replan_distance,
					  wavefront_buffer, carrot_distance,
					  carrot_stepsize, carrot_maxnsteps,
					  estar_step, coarse_grid_ncells,
					  plan_rwlock));
  }
  
  shared_ptr<TravmapCallback> travmap_cb(planner->GetTravmapCallback());
  
  shared_ptr<Mapper>
    mapper(new Mapper(m2d, swiped_map_update, max_swipe_distance, travmap_cb, grow_options.get()));
  if ( ! mapper) {
    cerr << "ERROR in Borox::InitAlgorithm():\n"
	 << "  Could not create asl::Mapper.\n";
    exit(EXIT_FAILURE);
  }
  
  shared_ptr<Controller> controller(new Controller(m_dwa_ptrack, planner));
  
  m_asl_algo.reset(new Algorithm(mapper, planner, controller));
}


void Borox::
InitScanners(boost::shared_ptr<sfl::Multiscanner> mscan,
	     smartparams const & params)
{
  mscan->Add(DefineLidar(sfl::Frame(params.front_mount_x,
				    params.front_mount_y,
				    params.front_mount_theta),
			 params.front_nscans,
			 params.front_rhomax,
			 params.front_phi0,
			 params.front_phirange,
			 params.front_channel)->GetScanner());
  mscan->Add(DefineLidar(sfl::Frame(params.rear_mount_x,
				    params.rear_mount_y,
				    params.rear_mount_theta),
			 params.rear_nscans,
			 params.rear_rhomax,
			 params.rear_phi0,
			 params.rear_phirange,
			 params.rear_channel)->GetScanner());
}


void Borox::
InitDrive(smartparams const & params)
{
  DefineDiffDrive(params.model_wheelbase,
		  params.model_wheelradius);
}


void Borox::
InitBody(smartparams const & params)
{
  if ( ! m_model)
    m_model = CreateModel(params);
  for(HullIterator ih(*m_model->GetHull()); ih.IsValid(); ih.Increment()){
    AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
    PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
  }
}


void Borox::
MoreGraphics(std::string const & name,
	     World const & world,
	     bool slow_drawing_enabled)
{
  AddDrawing(new DWDrawing(name + "_dwdrawing", *m_dwa));
  AddDrawing(new ODrawing(name + "_dodrawing",
			  m_dwa->GetDistanceObjective(),
			  m_dwa));
  AddDrawing(new ODrawing(name + "_hodrawing",
			  m_dwa->GetHeadingObjective(),
			  m_dwa));
  AddDrawing(new ODrawing(name + "_sodrawing",
			  m_dwa->GetSpeedObjective(),
			  m_dwa));
  AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				 *m_odo,
				 m_model->WheelBase() / 2));
  AddDrawing(new DODrawing(name + "_collisiondrawing",
			   m_dwa->GetDistanceObjective(),
			   m_dwa->GetHeadingObjective(),
			   m_dwa,
			   m_model));
  
  AddCamera(new StillCamera(name + "_dwcamera",
			    0,
			    0,
			    m_dwa->Dimension(),
			    m_dwa->Dimension(),
			    Instance<UniqueManager<Camera> >()));
  AddCamera(new OCamera(name + "_ocamera", *m_dwa));

  double a, b, c, d;
  m_dwa->GetDistanceObjective()->GetRange(a, b, c, d);
  AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d,
			    Instance<UniqueManager<Camera> >()));
}


const asl::trajectory_t * Borox::
GetTrajectory() const
{
#warning Should be in base class.
  return 0;
}

bool Borox::
GetRefpoint(asl::path_point &ref_point) const
{
#warning Should be in base class.
  return false;
}
