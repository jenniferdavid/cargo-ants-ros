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


#include "expo_parameters.hpp"
#include "../util/OptionDictionary.hpp"
#include "../util/strutil.hpp"
#include <cmath>

#ifdef WIN32
# include "../util/numeric.hpp"
#endif // WIN32

using namespace sfl;

namespace expo {

expo_parameters::
expo_parameters()
{
  expo_default_parameters(this);
}


expo_parameters::
expo_parameters(boost::shared_ptr<sfl::OptionDictionary> opt)
{
  expo_default_parameters(this);
  expo_parameters_load(*this, opt);
}


void expo_default_parameters(struct expo_parameters * params)
{
  params->model_security_distance = 0.02;
  params->model_wheelbase         = 0.521;
  params->model_wheelradius       = 0.088;
  params->model_qd_max            = 6.5;
  params->model_qdd_max           = 6.5;
  params->model_sd_max            = 0.5;
  params->model_thetad_max        = 2.0;
  params->model_sdd_max =
    0.75 * params->model_wheelradius * params->model_qd_max;
  params->model_thetadd_max =
    1.5 * params->model_wheelradius * params->model_qd_max
    / params->model_wheelbase;
  
  params->dwa_dimension       = 41;
  params->dwa_grid_width      = 2.3;
  params->dwa_grid_height     = 1.7;
  params->dwa_grid_resolution = 0.03;
  params->dwa_alpha_distance  = 0.6;
  params->dwa_alpha_heading   = 0.3;
  params->dwa_alpha_speed     = 0.1;
  params->dwa_use_tobi_distobj = false;
  params->dwa_tobi_distobj_blur = 5;
  
  params->bband_enabled           = true;
  params->bband_shortpath         = 2;
  params->bband_longpath          = 10;
  params->bband_maxignoredistance = 4;
  
  params->mp_dtheta_starthoming = 10 * M_PI / 180;
  params->mp_dtheta_startaiming = 45 * M_PI / 180;
}


void expo_parameters_load(expo_parameters & params,
			  boost::shared_ptr<sfl::OptionDictionary> opt)
{
  string_to(opt->GetOption("model_security_distance"), params.model_security_distance);
  string_to(opt->GetOption("model_wheelbase"), params.model_wheelbase);
  string_to(opt->GetOption("model_wheelradius"), params.model_wheelradius);
  string_to(opt->GetOption("model_qd_max"), params.model_qd_max);
  string_to(opt->GetOption("model_qdd_max"), params.model_qdd_max);
  string_to(opt->GetOption("model_sd_max"), params.model_sd_max);
  string_to(opt->GetOption("model_thetad_max"), params.model_thetad_max);
  string_to(opt->GetOption("model_sdd_max"), params.model_sdd_max);
  string_to(opt->GetOption("model_thetadd_max"), params.model_thetadd_max);
  
  string_to(opt->GetOption("dwa_dimension"), params.dwa_dimension);
  string_to(opt->GetOption("dwa_grid_width"), params.dwa_grid_width);
  string_to(opt->GetOption("dwa_grid_height"), params.dwa_grid_height);
  string_to(opt->GetOption("dwa_grid_resolution"), params.dwa_grid_resolution);
  string_to(opt->GetOption("dwa_alpha_distance"), params.dwa_alpha_distance);
  string_to(opt->GetOption("dwa_alpha_heading"), params.dwa_alpha_heading);
  string_to(opt->GetOption("dwa_alpha_speed"), params.dwa_alpha_speed);
  string_to_bool(opt->GetOption("dwa_use_tobi_distobj"), params.dwa_use_tobi_distobj);
  string_to(opt->GetOption("dwa_tobi_distobj_blur"), params.dwa_tobi_distobj_blur);
  
  string_to_bool(opt->GetOption("bband_enabled"), params.bband_enabled);
  string_to(opt->GetOption("bband_shortpath"), params.bband_shortpath);
  string_to(opt->GetOption("bband_longpath"), params.bband_longpath);
  string_to(opt->GetOption("bband_maxignoredistance"), params.bband_maxignoredistance);
  
  string_to(opt->GetOption("mp_dtheta_starthoming"), params.mp_dtheta_starthoming);
  string_to(opt->GetOption("mp_dtheta_startaiming"), params.mp_dtheta_startaiming);
}

}
