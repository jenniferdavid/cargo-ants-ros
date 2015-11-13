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
#include "robox_parameters.hpp"
#include "VisualRobox.hpp"
#include <npm/RobotServer.hpp>
#include <npm/World.hpp>
#include <npm/Lidar.hpp>
#include <npm/DiffDrive.hpp>
#include <npm/pdebug.hpp>
#include <sfl/util/strutil.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/expo/expo_parameters.hpp>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/MotionController.hpp>
#include <sfl/expo/Robox.hpp>


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


namespace local {
  
  class MPKeyListener: public KeyListener {
  public:
    MPKeyListener(shared_ptr<expo::MotionPlanner> _mp)
      : mp(_mp), stopped(false) {}
    
    virtual void KeyPressed(unsigned char key)
    {
      if('m' != key)
	return;
      if(stopped){
	stopped = false;
	mp->ManualResume();
      }
      else{
	stopped = true;
	mp->ManualStop();
      }
    }
    
    shared_ptr<expo::MotionPlanner> mp;
    bool stopped;
  };
  
  
  class NGKeyListener: public KeyListener {
  public:
    NGKeyListener(): next_goal(false) {}
    
    virtual void KeyPressed(unsigned char key)
    { if('n' == key) next_goal = true; }
    
    bool next_goal;
  };
  
}


scanner_desc_s::
scanner_desc_s()
  : nscans(181),
    mount_x(0.15),
    mount_y(0),
    mount_theta(0),
    rhomax(8),
    phi0(-M_PI/2),
    phirange(M_PI)
{
}


//// also to be recycled for customizability
//
// Robox::
// Robox(std::string const &name,
//       boost::shared_ptr<sfl::Hull> hull,
//       std::map<int, scanner_desc_s> const & scanners)
//   : RobotClient(name),
//     m_ngkl(new local::NGKeyListener())
// {
//   boost::shared_ptr<sfl::Multiscanner> mscan(new Multiscanner(GetHAL()));
//   for (std::map<int, scanner_desc_s>::const_iterator iscan(scanners.begin());
//        iscan != scanners.end(); ++iscan) {
//     mscan->Add(DefineLidar(Frame(iscan->second.mount_x,
// 				 iscan->second.mount_y,
// 				 iscan->second.mount_theta),
// 			   iscan->second.nscans,
// 			   iscan->second.rhomax,
// 			   iscan->second.phi0,
// 			   iscan->second.phirange,
// 			   iscan->first)->GetScanner());
//   }
//  
//   expo_parameters params(descriptor);
//   m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
//  
//   m_imp.reset(new npm::VisualRobox(descriptor->name, params, hull, GetHAL(), mscan));
//  
//   for (HullIterator ih(*m_imp->hull); ih.IsValid(); ih.Increment()) {
//     AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
//     PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
// 	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
//   }
//  
//   world.AddKeyListener(m_ngkl);
//   shared_ptr<KeyListener> listener(new local::MPKeyListener(m_imp->motionPlanner));
//   world.AddKeyListener(listener);
// }


Robox::
Robox(std::string const &name)
  : RobotClient(name),
    m_ngkl(new local::NGKeyListener())
{
  reflectParameter("model_security_distance", &m_params.model_security_distance);
  reflectParameter("model_wheelbase", &m_params.model_wheelbase);
  reflectParameter("model_wheelradius", &m_params.model_wheelradius);
  reflectParameter("model_qd_max", &m_params.model_qd_max);
  reflectParameter("model_qdd_max", &m_params.model_qdd_max);
  reflectParameter("model_sd_max", &m_params.model_sd_max);
  reflectParameter("model_thetad_max", &m_params.model_thetad_max);
  reflectParameter("model_sdd_max", &m_params.model_sdd_max);
  reflectParameter("model_thetadd_max", &m_params.model_thetadd_max);  
  reflectParameter("dwa_dimension", &m_params.dwa_dimension);
  reflectParameter("dwa_grid_width", &m_params.dwa_grid_width);
  reflectParameter("dwa_grid_height", &m_params.dwa_grid_height);
  reflectParameter("dwa_grid_resolution", &m_params.dwa_grid_resolution);
  reflectParameter("dwa_alpha_distance", &m_params.dwa_alpha_distance);
  reflectParameter("dwa_alpha_heading", &m_params.dwa_alpha_heading);
  reflectParameter("dwa_alpha_speed", &m_params.dwa_alpha_speed);
  reflectParameter("dwa_use_tobi_distobj", &m_params.dwa_use_tobi_distobj);
  reflectParameter("dwa_tobi_distobj_blur", &m_params.dwa_tobi_distobj_blur);
  reflectParameter("bband_enabled", &m_params.bband_enabled);
  reflectParameter("bband_shortpath", &m_params.bband_shortpath);
  reflectParameter("bband_longpath", &m_params.bband_longpath);
  reflectParameter("bband_maxignoredistance", &m_params.bband_maxignoredistance);
  reflectParameter("mp_dtheta_starthoming", &m_params.mp_dtheta_starthoming);
  reflectParameter("mp_dtheta_startaiming", &m_params.mp_dtheta_startaiming);
  reflectParameter("front_nscans", &m_params.front_nscans);
  reflectParameter("front_mount_x", &m_params.front_mount_x);
  reflectParameter("front_mount_y", &m_params.front_mount_y);
  reflectParameter("front_mount_theta", &m_params.front_mount_theta);
  reflectParameter("front_rhomax", &m_params.front_rhomax);
  reflectParameter("front_phi0", &m_params.front_phi0);
  reflectParameter("front_phirange", &m_params.front_phirange);
  reflectParameter("rear_nscans", &m_params.rear_nscans);
  reflectParameter("rear_mount_x", &m_params.rear_mount_x);
  reflectParameter("rear_mount_y", &m_params.rear_mount_y);
  reflectParameter("rear_mount_theta", &m_params.rear_mount_theta);
  reflectParameter("rear_rhomax", &m_params.rear_rhomax);
  reflectParameter("rear_phi0", &m_params.rear_phi0);
  reflectParameter("rear_phirange", &m_params.rear_phirange);
}


bool Robox::
Initialize(npm::RobotServer &server)
{
  if ( !npm::RobotClient::Initialize(server))
    return false;
  
  boost::shared_ptr<sfl::Hull> hull(expo::Robox::CreateDefaultHull());
  boost::shared_ptr<sfl::LocalizationInterface> localization(server.CreateFakeLocalization());
  
  Frame const front_mount(m_params.front_mount_x,
			  m_params.front_mount_y,
			  m_params.front_mount_theta);
  boost::shared_ptr<sfl::LidarChannel>
    front_channel = server.DefineLidar(front_mount,
				       "front_lidar",
				       m_params.front_nscans,
				       m_params.front_rhomax,
				       m_params.front_phi0,
				       m_params.front_phirange)->CreateChannel();
  boost::shared_ptr<sfl::Scanner> front(new sfl::Scanner(localization,
							 front_channel,
							 front_mount,
							 m_params.front_nscans,
							 m_params.front_rhomax,
							 m_params.front_phi0,
							 m_params.front_phirange));
  
  Frame const rear_mount(m_params.rear_mount_x,
			 m_params.rear_mount_y,
			 m_params.rear_mount_theta);
  boost::shared_ptr<sfl::LidarChannel>
    rear_channel = server.DefineLidar(rear_mount,
				      "rear_lidar",
				      m_params.rear_nscans,
				      m_params.rear_rhomax,
				      m_params.rear_phi0,
				      m_params.rear_phirange)->CreateChannel();
  boost::shared_ptr<sfl::Scanner> rear(new sfl::Scanner(localization,
							rear_channel,
							rear_mount,
							m_params.rear_nscans,
							m_params.rear_rhomax,
							m_params.rear_phi0,
							m_params.rear_phirange));
  
  boost::shared_ptr<sfl::Multiscanner> mscan(new Multiscanner(localization));
  mscan->Add(front);
  mscan->Add(rear);
  
  m_drive = server.DefineDiffDrive(m_params.model_wheelbase, m_params.model_wheelradius);
  
  m_imp.reset(new npm::VisualRobox(name, m_params, hull,
				   server.CreateFakeLocalization(),
				   m_drive->CreateChannel(),
				   mscan));
  
  for (HullIterator ih(*m_imp->hull); ih.IsValid(); ih.Increment()) {
    server.AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
    PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
  }
  
  server.AddKeyListener(m_ngkl);
  shared_ptr<KeyListener> listener(new local::MPKeyListener(m_imp->motionPlanner));
  server.AddKeyListener(listener);
  
  return true;
}


//// resurrect the custom robox using fpplib::configurable
//
// Robox * Robox::
// CreateCustom(shared_ptr<RobotDescriptor> descriptor, const World & world)
// {
//   try {
//     std::map<int, scanner_desc_s> scanners;
//     boost::scoped_ptr<sfl::Polygon> polygon;
//     boost::shared_ptr<sfl::Hull> hull;
//     typedef RobotDescriptor::custom_line_t::const_iterator line_it_t;
//     line_it_t iline(descriptor->GetCustomLines().begin());
//     line_it_t endline(descriptor->GetCustomLines().end());
//     for (/**/; iline != endline; ++iline) {
//       std::string lno("line " + sfl::to_string(iline->first) + ": ");
//       std::istringstream is(iline->second);
//       PDEBUG ("line %d: %s\n", iline->first, iline->second.c_str());
//       string token;
//       is >> token;
//       if (token == "scanner") {
// 	int channel;
// 	string keyword;
// 	is >> channel >> keyword;
// 	if ( ! is) {
// 	  throw std::runtime_error(lno + "invalid channel or keyword for scanner");
// 	}
// 	if (keyword == "mount") {
// 	  double xx, yy, theta;
// 	  is >> xx >> yy >> theta;
// 	  if ( ! is) {
// 	    throw std::runtime_error(lno + "invalid mount for scanner");
// 	  }
// 	  scanners[channel].mount_x = xx;
// 	  scanners[channel].mount_y = yy;
// 	  scanners[channel].mount_theta = theta;
// 	  PDEBUG ("scanner[%d].mount = %g %g %g\n", channel, xx, yy, theta);
// 	}
// 	else if (keyword == "nscans") {
// 	  int nscans;
// 	  is >> nscans;
// 	  if (( ! is) || (0 >= nscans)) {
//             throw std::runtime_error(lno + "invalid nscans for scanner");
//           }
// 	  scanners[channel].nscans = nscans;
// 	  PDEBUG ("scanner[%d].nscans = %d\n", channel, nscans);
// 	}
// 	else if (keyword == "rhomax") {
//           double rhomax;
//           is >> rhomax;
//           if (( ! is) || (0 >= rhomax)) {
//             throw std::runtime_error(lno + "invalid rhomax for scanner");
//           }
//           scanners[channel].rhomax = rhomax;
// 	  PDEBUG ("scanner[%d].rhomax = %g\n", channel, rhomax);
//         }
// 	else if (keyword == "phi") {
//           double phi0, phirange;
//           is >> phi0 >> phirange;
//           if ( ! is) {
//             throw std::runtime_error(lno + "invalid phi0 or phirange for scanner");
//           }
//           scanners[channel].phi0 = phi0;
//           scanners[channel].phirange = phirange;
// 	  PDEBUG ("scanner[%d].phi = %g %g\n", channel, phi0, phirange);
//         }
// 	else {
// 	  throw std::runtime_error(lno + "invalid keyword `" + keyword + "' for scanner");
// 	}
//       } // endif "scanner"
//       else if (token == "hull") {
// 	string keyword;
// 	is >> keyword;
// 	if ( ! is) {
//           throw std::runtime_error(lno + "invalid keyword for hull");
//         }
// 	if ((keyword == "point") || (keyword == "points")) {
// 	  if ( ! polygon) {
// 	    polygon.reset(new sfl::Polygon());
// 	  }
// 	  double xx, yy;
// 	  while (is >> xx >> yy) {
// 	    polygon->AddPoint(xx, yy);
// 	    PDEBUG ("hull point %g %g\n", xx, yy);
// 	  }
// 	}
// 	else if (keyword == "break") {
// 	  if (polygon) {
// 	    if ( ! hull) {
// 	      hull.reset(new sfl::Hull());
// 	    }
// 	    hull->AddPolygon(*polygon);
// 	    polygon.reset();
// 	  }
// 	  PDEBUG ("hull break\n");
// 	}
// 	else {
// 	  throw std::runtime_error(lno + "invalid keyword `" + keyword + "' for hull");
// 	}
//       } // endif "hull"
//       else if (token == "diffdrive") {
// 	double wheelbase, wheelradius;
// 	is >> wheelbase >> wheelradius;
// 	if ( ! is) {
// 	  throw std::runtime_error(lno + "invalid diffdrive");
// 	}
// 	descriptor->SetOption("model_wheelbase", sfl::to_string(wheelbase));
// 	descriptor->SetOption("model_wheelradius", sfl::to_string(wheelradius));
// 	PDEBUG ("diffdrive %g %g\n", wheelbase, wheelradius);
//       } // endif "diffdrive"
//       else {
// 	throw std::runtime_error(lno + "invalid token `" + token + "'");
//       }
//     } // end for line
    
//     // make sure we have at least one laser scanner
//     if (scanners.empty()) {
//       scanners[0] = scanner_desc_s();
//       PDEBUG ("added default scanner\n");
//     }
    
//     // make sure we finalize the last (possibly only) polygon
//     if (polygon) {
//       if ( ! hull) {
// 	hull.reset(new sfl::Hull());
//       }
//       hull->AddPolygon(*polygon);
//       PDEBUG ("finalized hull\n");
//     }
    
//     // make sure we have a hull
//     if ( ! hull) {
//       hull = expo::Robox::CreateDefaultHull();
//       PDEBUG ("set default hull\n");
//     }
    
//     Robox * robox(new Robox(descriptor, world, hull, scanners));
//     if( ! robox->StartThreads()){
//       std::cerr << "ERROR in Robox::CreateCustom(): failed to StartThreads()\n";
//       delete robox;
//       return 0;
//     }
//     return robox;
    
//   }
//   catch (std::runtime_error const & ee) {
//     std::cerr << "ERROR in Robox::CreateCustom(): " << ee.what() << "\n";
//     return 0;
//   }
//   return 0;			// we never get here though.
// }


void Robox::
InitPose(sfl::Frame const &pp)
{
  m_imp->odometry->Clear();
  m_imp->odometry->Update(sfl::Pose(pp.X(), pp.Y(), pp.Theta()));
}


void Robox::
SetPose(sfl::Frame const &pp)
{
  m_imp->odometry->Update(sfl::Pose(pp.X(), pp.Y(), pp.Theta()));
}


bool Robox::
GetPose(sfl::Frame &pp)
  const
{
  boost::shared_ptr<const Pose> pose(m_imp->odometry->Get());
  if (!pose)
    return false;
  pp = *pose;
  return true;
}


bool Robox::
GetGoal(sfl::Goal &goal)
  const
{
  goal = m_imp->GetGoal();
  return true;
}


bool Robox::
GoalReached()
  const
{
  if(m_ngkl->next_goal){
    m_ngkl->next_goal = false;
    return true;
  }
  return m_imp->motionPlanner->GoalReached();
}


void Robox::
SetGoal(double timestep, const Goal & goal)
{
  m_imp->SetGoal(timestep, goal);
}


bool Robox::
PrepareAction(double timestep)
{
  try {
    m_imp->Update(timestep);
  }
  catch (runtime_error ee) {
    cerr << "Robox::PrepareAction(): m_imp->Update() failed with exception\n"
	 << ee.what() << "\n";
    return false;
  }
  return true;
}
