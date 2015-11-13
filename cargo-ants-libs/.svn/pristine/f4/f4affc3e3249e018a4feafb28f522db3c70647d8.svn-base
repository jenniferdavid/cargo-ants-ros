/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
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

#include "SmartCarmen.hpp"
#include "wrap_carmen.hpp"
#include "smart_params.hpp"
#include <sfl/api/Goal.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/util/Pthread.hpp>
#include <sfl/util/Line.hpp>
#include "../common/Lidar.hpp"
#include "../common/HAL.hpp"
#include "../common/RobotServer.hpp"
#include "../common/RobotDescriptor.hpp"
#include "../common/pdebug.hpp"
#include <iostream>


using namespace npm;
using namespace wrap_carmen;
using namespace sfl;
using namespace boost;
using namespace std;


SmartCarmen::
SmartCarmen(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, 2, true)
{
  smartparams params(descriptor);
  m_nscans = params.front_nscans;
  m_sick_channel = params.front_channel;
	
  double wheelbase(params.model_wheelbase);
  double wheelradius(params.model_wheelradius);
  double axlewidth(params.model_axlewidth);
	
	m_mscan.reset(new Multiscanner(GetHAL()));
	m_sick = DefineLidar(Frame(params.front_mount_x,
														 params.front_mount_y,
														 params.front_mount_theta),
											 params.front_nscans,
											 params.front_rhomax,
											 params.front_phi0,
											 params.front_phirange,
											 params.front_channel)->GetScanner();
	m_mscan->Add(m_sick);
	
	DefineBicycleDrive(wheelbase, wheelradius, axlewidth);

  AddLine(Line(-wheelradius, -axlewidth/2 -wheelradius,
							 -wheelradius,  axlewidth/2 +wheelradius));
  AddLine(Line(wheelbase +wheelradius, -axlewidth/2 -wheelradius,
							 wheelbase +wheelradius,  axlewidth/2 +wheelradius));
  AddLine(Line(-wheelradius, -axlewidth/2 -wheelradius,
							 wheelbase +wheelradius, -axlewidth/2 -wheelradius));
  AddLine(Line(-wheelradius,  axlewidth/2 +wheelradius,
							 wheelbase +wheelradius,  axlewidth/2 +wheelradius));
	
	if( ! init_receive_steering(&cerr)){
		cerr << "ERROR in SmartCarmen ctor:\n"
				 << "  wrap_carmen::init_receive_steering() failed.\n";
		exit(EXIT_FAILURE);
	}
	if( ! init_send_pos(&cerr)){
		cerr << "ERROR in SmartCarmen ctor:\n"
				 << "  wrap_carmen::init_send_pos() failed.\n";
		exit(EXIT_FAILURE);
	}
	if( ! init_send_laser(&cerr)){
		cerr << "ERROR in SmartCarmen ctor:\n"
				 << "  wrap_carmen::init_send_laser() failed.\n";
		exit(EXIT_FAILURE);
	}	
}


SmartCarmen::
~SmartCarmen()
{
	if( ! cleanup_receive_steering(&cerr))
		cerr << "WARNING in SmartCarmen dtor:\n"
				 << "  wrap_carmen::cleanup_receive_steering() failed.\n";
	if( ! cleanup_send_pos(&cerr))
		cerr << "WARNING in SmartCarmen dtor:\n"
				 << "  wrap_carmen::cleanup_send_pos() failed.\n";
	if( ! cleanup_send_laser(&cerr))
		cerr << "WARNING in SmartCarmen dtor:\n"
				 << "  wrap_carmen::cleanup_send_laser() failed.\n";
}


bool SmartCarmen::
PrepareAction(double timestep)
{
	PDEBUG("\n\n==================================================\n");

	m_mscan->UpdateAll();
	if( ! send_laser(*m_sick, &cerr)){
		cerr << "ERROR in SmartCarmen PrepareAction:\n"
				 << "  wrap_carmen::send_laser() failed.\n";
		return false;
	}
	
	double x, y, theta, velocity, steering;
	GetPose(x, y, theta);
	
	double qd[2];
	size_t len(2);
	if ((0 != GetHAL()->speed_get(qd, &len)) || (2 != len)) {
		cerr << "ERROR in SmartCarmen PrepareAction:\n"
				 << "  GetHAL()->speed_get() failed.\n";
		return false;
	}
	velocity = qd[0];
	steering = qd[1];
	
	if( ! send_pos(x, y, theta, velocity, steering, &cerr)){
		cerr << "ERROR in SmartCarmen PrepareAction:\n"
				 << "  wrap_carmen::send_pos() failed.\n";
		return false;
	}
	
	if( ! receive_steering(velocity, steering, &cerr)){
		cerr << "ERROR in SmartCarmen PrepareAction:\n"
				 << "  wrap_carmen::receive_steering() failed.\n";
		return false;
	}
	
	qd[0] = velocity;
	qd[1] = steering;
	if ((0 != GetHAL()->speed_set(qd, &len)) || (2 != len)) {
		cerr << "ERROR in SmartCarmen PrepareAction:\n"
				 << "  GetHAL()->speed_set() failed.\n";
		return false;
	}
	
	return true;
}


void SmartCarmen::
InitPose(double x, double y, double theta)
{
}


void SmartCarmen::
SetPose(double x, double y, double theta)
{
}


void SmartCarmen::
GetPose(double & x, double & y, double & theta)
{
  const Frame & pose(GetServer()->GetTruePose());
  x = pose.X();
  y = pose.Y();
  theta = pose.Theta();
}


void SmartCarmen::
SetGoal(double timestep, const sfl::Goal & goal)
{
	if(m_goal)
		*m_goal = goal;
	else
		m_goal.reset(new Goal(goal));
}


shared_ptr<const Goal> SmartCarmen::
GetGoal()
{
  return m_goal;
}


bool SmartCarmen::
GoalReached()
{
	if(m_goal)
		return m_goal->Reached(GetServer()->GetTruePose(), true);
	return false;
}
