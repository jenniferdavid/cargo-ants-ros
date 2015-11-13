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


#include "robox_parameters.hpp"
#include <sfl/util/OptionDictionary.hpp>
#include <sfl/util/strutil.hpp>
#include <cmath>

#ifdef WIN32
# include <sfl/util/numeric.hpp>
#endif // WIN32

using namespace sfl;


robox_parameters::
robox_parameters()
{
  robox_default_parameters(this);
}


robox_parameters::
robox_parameters(boost::shared_ptr<sfl::OptionDictionary> opt)
  : expo_parameters(opt)
{
  robox_default_parameters(this);
  robox_parameters_load(*this, opt);
}


void robox_default_parameters(struct robox_parameters * params)
{
  expo_default_parameters(params);
  
  params->front_nscans      = 181;
  params->front_mount_x     = 0.15;
  params->front_mount_y     = 0;
  params->front_mount_theta = 0;
  params->front_rhomax      = 8;
  params->front_phi0        = -M_PI/2;
  params->front_phirange    = M_PI;
  
  params->rear_nscans       = 181;
  params->rear_mount_x      = -0.15;
  params->rear_mount_y      = 0;
  params->rear_mount_theta  = M_PI;
  params->rear_rhomax       = 8;
  params->rear_phi0         = -M_PI/2;
  params->rear_phirange     = M_PI;
}


void robox_parameters_load(robox_parameters & params,
			  boost::shared_ptr<sfl::OptionDictionary> opt)
{
  expo_parameters_load(params, opt);

  string_to(opt->GetOption("front_nscans"), params.front_nscans);
  string_to(opt->GetOption("front_mount_x"), params.front_mount_x);
  string_to(opt->GetOption("front_mount_y"), params.front_mount_y);
  string_to(opt->GetOption("front_mount_theta"), params.front_mount_theta);
  string_to(opt->GetOption("front_rhomax"), params.front_rhomax);
  string_to(opt->GetOption("front_phi0"), params.front_phi0);
  string_to(opt->GetOption("front_phirange"), params.front_phirange);
  
  string_to(opt->GetOption("rear_nscans"), params.rear_nscans);
  string_to(opt->GetOption("rear_mount_x"), params.rear_mount_x);
  string_to(opt->GetOption("rear_mount_y"), params.rear_mount_y);
  string_to(opt->GetOption("rear_mount_theta"), params.rear_mount_theta);
  string_to(opt->GetOption("rear_rhomax"), params.rear_rhomax);
  string_to(opt->GetOption("rear_phi0"), params.rear_phi0);
  string_to(opt->GetOption("rear_phirange"), params.rear_phirange);
}
