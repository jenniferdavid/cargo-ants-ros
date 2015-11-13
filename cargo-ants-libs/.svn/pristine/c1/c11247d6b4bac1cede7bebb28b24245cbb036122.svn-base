/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "Robox.hpp"
#include "MotionPlanner.hpp"
#include "MotionController.hpp"
#include "expo_parameters.hpp"

#include "../api/Odometry.hpp"
#include "../api/Multiscanner.hpp"
#include "../api/RobotModel.hpp"
#include "../dwa/DynamicWindow.hpp"
#include "../dwa/DistanceObjective.hpp"
#include "../dwa/DistanceObjectiveToBI.hpp"
#include "../dwa/SpeedObjective.hpp"
#include "../dwa/HeadingObjective.hpp"
#include "../bband/BubbleBand.hpp"
#include "MotionPlanner.hpp"
#include "MotionController.hpp"
#include <sstream>

using namespace boost;
using namespace std;


namespace expo {
  
  
  Robox::
  Robox(expo_parameters const & params,
	boost::shared_ptr<sfl::Hull> _hull,
	boost::shared_ptr<sfl::LocalizationInterface> localization,
	shared_ptr<sfl::DiffDriveChannel> drive,
	shared_ptr<sfl::Multiscanner> _mscan)
    : hull(_hull),
      mscan(_mscan)
  {
    sfl::RobotModel::Parameters const
      modelParms(params.model_security_distance,
		 params.model_wheelbase,
		 params.model_wheelradius,
		 params.model_qd_max,
		 params.model_qdd_max,
		 params.model_sd_max,
		 params.model_thetad_max,
		 params.model_sdd_max,
		 params.model_thetadd_max);
    robotModel.reset(new sfl::RobotModel(modelParms, hull));
    motionController.
      reset(new MotionController(robotModel, drive));
    odometry.reset(new sfl::Odometry(localization));
    if (params.bband_enabled) {
      bubbleBand.
	reset(new sfl::BubbleBand(*robotModel, *odometry, *mscan,
				  sfl::BubbleList::Parameters(params.bband_shortpath,
							      params.bband_longpath,
							      params.bband_maxignoredistance)));
    }
    
    if ( ! params.dwa_use_tobi_distobj) {
      sfl::LegacyDynamicWindow * dwa(new sfl::LegacyDynamicWindow(params.dwa_dimension,
								  params.dwa_grid_width,
								  params.dwa_grid_height,
								  params.dwa_grid_resolution,
								  robotModel,
								  params.dwa_alpha_distance,
								  params.dwa_alpha_heading,
								  params.dwa_alpha_speed,
								  true));
      dynamicWindow.reset(dwa);
      distanceObjective = dwa->GetDistanceObjective();
      speedObjective = dwa->GetSpeedObjective();
      headingObjective = dwa->GetHeadingObjective();
    }
    else {
      dynamicWindow.reset(new sfl::DynamicWindow(params.dwa_dimension,
						 robotModel));
      distanceObjective.reset(new sfl::DistanceObjectiveToBI(*dynamicWindow,
							     robotModel,
							     params.dwa_grid_width,
							     params.dwa_grid_height,
							     params.dwa_grid_resolution,
							     params.dwa_tobi_distobj_blur));
      dynamicWindow->AddObjective(distanceObjective, params.dwa_alpha_distance);
      headingObjective.reset(new sfl::HeadingObjective(*dynamicWindow, *robotModel));
      dynamicWindow->AddObjective(headingObjective, params.dwa_alpha_heading);
      speedObjective.reset(new sfl::SpeedObjective(*dynamicWindow, *robotModel));
      dynamicWindow->AddObjective(speedObjective, params.dwa_alpha_speed);
      dynamicWindow->Initialize(&cout);
    }
    
    motionPlanner.reset(new MotionPlanner(motionController,
					  dynamicWindow,
					  speedObjective,
					  headingObjective,
					  mscan,
					  robotModel,
					  bubbleBand,
					  odometry));
    if ((params.mp_dtheta_starthoming > 0) && (params.mp_dtheta_startaiming > 0)) {
      if ( ! motionPlanner->SetAimingThresholds(params.mp_dtheta_startaiming,
						params.mp_dtheta_starthoming)) {
	cerr << "WARNING in expo::Robox constructor:\n"
	     << "  motionPlanner->SetAimingThresholds("
	     << params.mp_dtheta_startaiming << ", "
	     << params.mp_dtheta_starthoming << ") failed\n";
      }
    }
  }
  
  
  shared_ptr<sfl::Hull> Robox::
  CreateDefaultHull()
  {
    shared_ptr<sfl::Hull> hull(new sfl::Hull());
    static const double octoSmall = 0.178;
    static const double octoBig = 0.430;
    
    sfl::Polygon outline;
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
  
  
  void Robox::
  SetGoal(double timestep, sfl::Goal const & goal)
  {
    motionPlanner->SetGoal(timestep, goal);
  }
  
  
  sfl::Goal const & Robox::
  GetGoal() const
  {
    return motionPlanner->GetGoal();
  }
  
  
  void Robox::
  Update(double timestep) throw(std::runtime_error)
  {
    ostringstream os;
    if ( ! UpdateMultiscanner(&os)) {
      os << "expo::Robox::Update(): UpdateMultiscanner() failed";
      throw runtime_error(os.str());
    }
    if ( ! UpdateMotionPlanner(timestep)) {
      os << "expo::Robox::Update(): UpdateMotionPlanner() failed";
      throw runtime_error(os.str());
    }
    int status(UpdateMotionController(timestep, &os));
    if (0 != status) {
      os << "expo::Robox::Update(): UpdateMotionController failed with status " << status;
      throw runtime_error(os.str());
    }
    status = odometry->Update();
    if (0 != status) {
      os << "expo::Robox::Update(): UpdateOdometry failed with status " << status;
      throw runtime_error(os.str());
    }
  }
  
  
  bool Robox::
  UpdateMultiscanner(std::ostream * err_os)
  {
    return mscan->UpdateAll(err_os);
  }
  
  
  bool Robox::
  UpdateMotionPlanner(double timestep)
  {
    motionPlanner->Update(timestep);
    return MotionPlanner::invalid != motionPlanner->GetStateId();
  }
  
  
  int Robox::
  UpdateMotionController(double timestep, std::ostream * err_os)
  {
    return motionController->Update(timestep, err_os);
  }

}
