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


#ifndef ROBOX_PARAMETERS_HPP
#define ROBOX_PARAMETERS_HPP

#include <sfl/expo/expo_parameters.hpp>

struct robox_parameters
  : public expo::expo_parameters
{
  robox_parameters();
  
  explicit robox_parameters(boost::shared_ptr<sfl::OptionDictionary> opt);
  
  int front_nscans;
  double front_mount_x, front_mount_y, front_mount_theta;
  double front_rhomax, front_phi0, front_phirange;
  
  int rear_nscans;
  double rear_mount_x, rear_mount_y, rear_mount_theta;
  double rear_rhomax, rear_phi0, rear_phirange;
};

void robox_default_parameters(struct robox_parameters * params);

void robox_parameters_load(robox_parameters & params,
			   boost::shared_ptr<sfl::OptionDictionary> opt);

#endif // ROBOX_PARAMETERS_HPP
