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

#ifndef SMARTPARAMS_HPP
#define SMARTPARAMS_HPP

#include <sfl/expo/expo_parameters.h>

struct smartparams
  : public expo_parameters
{
  explicit smartparams(boost::shared_ptr<sfl::OptionDictionary> opt);

  double model_axlewidth;
  double model_phi_max;
  double model_phid_max;

  int front_channel, front_nscans;
  double front_mount_x, front_mount_y, front_mount_theta;
  double front_rhomax, front_phi0, front_phirange;
  
  int rear_channel, rear_nscans;
  double rear_mount_x, rear_mount_y, rear_mount_theta;
  double rear_rhomax, rear_phi0, rear_phirange;
};

#endif // SMARTPARAMS_HPP
