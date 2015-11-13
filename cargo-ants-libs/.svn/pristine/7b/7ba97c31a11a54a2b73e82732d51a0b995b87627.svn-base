/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#ifndef NPM2_KINEMATIC_CONTROL_HPP
#define NPM2_KINEMATIC_CONTROL_HPP

#include <npm2/Process.hpp>
#include <npm2/GenericDrive.hpp>
#include <sfl/api/Goal.hpp>


namespace npm2 {
  
  
  /**
     Uses the non-holonomic velocity control method from (Siegwart and
     Nourbakhsh, Introduction to Autonomous Mobile Robots, MIT Press,
     ISBN 0-262-19502-X).
  */
  class KinematicControl
    : public Process
  {
  public:
    explicit KinematicControl (string const & name);
    
    bool setGoal (Goal const & goal);
    bool enable (bool enable);
    
    double kr_;			// default 3.0;  kr_ > 0.0
    double kd_;			// default -1.5; kd_ < 0.0
    double kg_;			// default 8.0;  kg_ + 5 kd_ / 3 - 2 kr_ / pi > 0.0
    double vtrans_max_;
    double vrot_max_;
    GenericDrive * drive_;
    
  protected:
    virtual state_t init (ostream & erros);
    virtual state_t run (double timestep, ostream & erros);
    
    bool enabled_;
    bool have_goal_;
    Goal goal_;
  };
  
}

#endif // NPM2_KINEMATIC_CONTROL_HPP
