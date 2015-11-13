/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#include "Smart.hpp"
#include "smart_params.hpp"
#include "ArcDrawing.hpp"
#include <asl/Algorithm.hpp>
#include <asl/Planner.hpp>
#include <asl/AckermannController.hpp>
#include <asl/ControlParams.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/util/strutil.hpp>
#include "../common/Lidar.hpp"
#include "../common/RobotDescriptor.hpp"
#include "../common/pdebug.hpp"
#include <iostream>


using namespace npm;
using namespace sfl;
using namespace asl;
using namespace boost;
using namespace std;


Smart::
Smart(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : AslBot(descriptor, world)
{
	CreateMePlease(descriptor, world); // calls various hooks...
}


void Smart::
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
	robot_radius = 1.5;
	string_to(descriptor->GetOption("robot_radius"), robot_radius);
	
	double buffer_zone(robot_radius);
	string_to(descriptor->GetOption("buffer_zone"), buffer_zone);
	
	string controller_name(descriptor->GetOption("controller_name"));
	if(controller_name == "")
		controller_name = "simple";
	
	bool use_simple_query(false);
	string_to(descriptor->GetOption("use_simple_query"), use_simple_query);

	shared_ptr<ControlParams> control_params(new ControlParams());
	bool use_default_control_params(true);
 	string_to(descriptor->GetOption("use_default_control_params"),
 						use_default_control_params);
	if ( ! use_default_control_params) {
		control_params->max_longitudinal_speed = params.model_sd_max;
		double control_sd_min_factor(0.5);
		string_to(descriptor->GetOption("control_sd_min_factor"),
							control_sd_min_factor);
		control_params->min_longitudinal_speed =
			control_sd_min_factor * params.model_sd_max;
		control_params->longitudinal_acc = params.model_sdd_max;
		control_params->min_steering_angle = - params.model_phi_max;
		control_params->max_steering_angle = params.model_phi_max;
		control_params->max_steering_rate = params.model_phid_max;
	}
	
	size_t coarse_grid_ncells(10);
	string_to(descriptor->GetOption("coarse_grid_ncells"), coarse_grid_ncells);
	
	ostringstream err_os;
	m_ackalgo.reset(asl::AckermannAlgorithm::
									 Create(robot_radius,
													buffer_zone,
													traversability_file,
													grow_options.get(),
													descriptor->GetOption("m2d_grow_strategy"),
													replan_distance,
													wavefront_buffer,
													carrot_distance,
													carrot_stepsize,
													carrot_maxnsteps,
													estar_step,
													! use_simple_query,	// use_estar = ! use_simple_q
													estar_grow_grid,
													coarse_grid_ncells,
													estar_options,
													swiped_map_update,
													max_swipe_distance,
													controller_name,
													control_params,
													AckermannModel(params.model_sd_max,
																				 params.model_sdd_max,
																				 params.model_phi_max,
																				 params.model_phid_max,
																				 params.model_wheelbase),
													goalmgr_filename,
													&err_os));
	if( ! m_ackalgo){
		cerr << "ERROR asl::AckermannAlgorithm::Create() failed\n "
				 << err_os.str() << "\n";
		exit(EXIT_FAILURE);
	}
	m_asl_algo = m_ackalgo;
	
 	m_acntrl = m_ackalgo->GetAckermannController();
	if( ! m_acntrl){
 		cerr << "ERROR m_ackalgo->GetAckermannController() failed\n";
 		exit(EXIT_FAILURE);
	}
}


void Smart::
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
}


void Smart::
InitDrive(smartparams const & params)
{
	DefineBicycleDrive(params.model_wheelbase,
										 params.model_wheelradius,
										 params.model_axlewidth);
}


void Smart::
InitBody(smartparams const & params)
{
  AddLine(Line(-params.model_wheelradius,
							 -params.model_axlewidth/2 -params.model_wheelradius,
							 -params.model_wheelradius,
							 params.model_axlewidth/2 +params.model_wheelradius));
  AddLine(Line(params.model_wheelbase +params.model_wheelradius,
							 -params.model_axlewidth/2 -params.model_wheelradius,
							 params.model_wheelbase +params.model_wheelradius,
							 params.model_axlewidth/2 +params.model_wheelradius));
  AddLine(Line(-params.model_wheelradius,
							 -params.model_axlewidth/2 -params.model_wheelradius,
							 params.model_wheelbase +params.model_wheelradius,
							 -params.model_axlewidth/2 -params.model_wheelradius));
  AddLine(Line(-params.model_wheelradius,
							 params.model_axlewidth/2 +params.model_wheelradius,
							 params.model_wheelbase +params.model_wheelradius,
							 params.model_axlewidth/2 +params.model_wheelradius));
}


void Smart::
MoreGraphics(std::string const & name,
						 World const & world,
						 bool slow_drawing_enabled)
{
	shared_ptr<ArcDrawing>
		ad(new ArcDrawing(name + "_arcs_cached", this,
											slow_drawing_enabled, false));
 	AddDrawing(ad);
	world.AddKeyListener(ad);
	ad.reset(new ArcDrawing(name + "_arcs_recomp", this,
													slow_drawing_enabled, true));
 	AddDrawing(ad);
	world.AddKeyListener(ad);
}


const asl::trajectory_t * Smart::
GetTrajectory() const
{
	return m_ackalgo->GetTrajectory();
}

bool Smart::
GetRefpoint(asl::path_point &ref_point) const
{
	return m_ackalgo->GetRefpoint(ref_point);
}


const asl::ArcControl * Smart::
GetArcControl() const
{
	return m_acntrl->GetArcControl();
}


boost::shared_ptr<const asl::NavFuncQuery> Smart::
GetQuery() const
{
	return m_asl_algo->GetPlanner()->GetNavFuncQuery();
}
